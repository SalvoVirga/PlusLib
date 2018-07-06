#include "vtkPlusSimplePublisher.h"
#include <vtkMatrix3x3.h>
#include <vtkMatrix4x4.h>
#include <vtkQuaternion.h>
#include "vtkPlusTrackedFrameList.h"

vtkStandardNewMacro(vtkPlusSimplePublisher);

//------------------------------------------------------------------------------
vtkPlusSimplePublisher::vtkPlusSimplePublisher() : Threader(vtkSmartPointer<vtkMultiThreader>::New()) {}

//------------------------------------------------------------------------------
PlusStatus vtkPlusSimplePublisher::Start(vtkPlusDataCollector* dataCollector,
                                         vtkPlusTransformRepository* transformRepository,
                                         vtkXMLDataElement* serverElement, const std::string& configFilePath) {
  if (serverElement == nullptr) {
    LOG_ERROR("NULL configuration sent to vtkPlusSimplePublisher::Start. Unable to start PlusServer.");
    return PLUS_FAIL;
  }

  SetDataCollector(dataCollector);
  if (ReadConfiguration(serverElement, configFilePath.c_str()) != PLUS_SUCCESS) {
    LOG_ERROR("Failed to read PlusSimplePublisher configuration");
    return PLUS_FAIL;
  }

  SetTransformRepository(transformRepository);

  if (StartSimpleService() != PLUS_SUCCESS) {
    LOG_ERROR("Failed to start Plus Simple publisher");
    return PLUS_FAIL;
  }

  return PLUS_SUCCESS;
}

//------------------------------------------------------------------------------
PlusStatus vtkPlusSimplePublisher::ReadConfiguration(vtkXMLDataElement* serverElement, const std::string& aFilename) {
  LOG_TRACE("vtkPlusSimplePublisher::ReadConfiguration");

  if (aFilename.empty()) {
    LOG_ERROR("Unable to configure PlusServer without an acceptable config file submitted.");
    return PLUS_FAIL;
  }
  SetConfigFilename(aFilename);

  XML_READ_SCALAR_ATTRIBUTE_REQUIRED(int, ListeningPort, serverElement);
  XML_READ_STRING_ATTRIBUTE_REQUIRED(OutputChannelId, serverElement);

  vtkXMLDataElement* defaultClientInfo = serverElement->FindNestedElementWithName("PublisherInfo");
  if (defaultClientInfo != nullptr) {
    // Get the message type to publish
    vtkXMLDataElement* messageTypes = defaultClientInfo->FindNestedElementWithName("MessageType");
    if (messageTypes != NULL) {
      if (messageTypes->GetNumberOfNestedElements() > 1) {
        LOG_WARNING("A SIMPLE Publisher can only publish one message type. Only the first one will be used.");
      }
      auto messageName = messageTypes->GetNestedElement(0)->GetName();
      if (messageName != nullptr && STRCASECMP(messageName, "Message") == 0) {
        vtkXMLDataElement* typeElem = messageTypes->GetNestedElement(0);
        XML_READ_STRING_ATTRIBUTE_NONMEMBER_REQUIRED(Type, PublisherMessageType, typeElem);
      }
    }

    if (PublisherMessageType == "TRANSFORM") {
      vtkXMLDataElement* transformNames = defaultClientInfo->FindNestedElementWithName("TransformNames");
      if (transformNames != nullptr) {
        for (int i = 0; i < transformNames->GetNumberOfNestedElements(); ++i) {
          const char* transform = transformNames->GetNestedElement(i)->GetName();
          if (transform != nullptr || STRCASECMP(transform, "Transform") == 0) {
            vtkXMLDataElement* transformElem = transformNames->GetNestedElement(i);
            std::string name{""};
            XML_READ_STRING_ATTRIBUTE_NONMEMBER_OPTIONAL(Name, name, transformElem);
            if (name.empty()) {
              LOG_WARNING("In TransformNames child transform #"
                          << i << " definition is incomplete: required Name attribute is missing.");
              continue;
            }

            PlusTransformName tName;
            if (tName.SetTransformName(name) != PLUS_SUCCESS) {
              LOG_WARNING("Invalid transform name: " << name);
              continue;
            }
            TransformNames.push_back(name);
          }
        }
      }
    } else if (PublisherMessageType == "IMAGE") {
      vtkXMLDataElement* imageNames = defaultClientInfo->FindNestedElementWithName("ImageNames");
      if (imageNames != nullptr) {
        for (int i = 0; i < imageNames->GetNumberOfNestedElements(); ++i) {
          const char* image = imageNames->GetNestedElement(i)->GetName();
          if (image == nullptr || STRCASECMP(image, "Image") != 0) { continue; }
          vtkXMLDataElement* imageElem = imageNames->GetNestedElement(i);
          std::string embeddedTransformToFrame{""};
          XML_READ_STRING_ATTRIBUTE_NONMEMBER_OPTIONAL(EmbeddedTransformToFrame, embeddedTransformToFrame, imageElem);
          if (embeddedTransformToFrame.empty()) {
            LOG_WARNING("EmbeddedTransformToFrame attribute of ImageNames/Image element #"
                        << i << " is missing. This element will be ignored.");
            continue;
          }

          std::string imageName;
          XML_READ_STRING_ATTRIBUTE_NONMEMBER_OPTIONAL(Name, imageName, imageElem);
          if (imageName.empty()) {
            LOG_WARNING("Name attribute of ImageNames/Image element # "
                        << i << " is missing. This element will be ignored.");
            continue;
          }
          ImagesNames.emplace_back(ImageStream(imageName, embeddedTransformToFrame));
          Publisher = simple::Publisher<simple_msgs::Image<uint8_t>>("tcp://*:" + std::to_string(ListeningPort));
        }
      }
    } else {
      LOG_ERROR("The wrong message type or none has been specified in the config file. IMAGE and TRANSFORM are the "
                "available message types.");
      return PLUS_FAIL;
    }
  }
  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusSimplePublisher::StartSimpleService() {
  if (DataCollector == nullptr) {
    LOG_WARNING("Tried to start SIMPLE publisher without a vtkPlusDataCollector");
    return PLUS_FAIL;
  }

  // If no thread was every spawned yet.
  if (DataSenderThreadId < 0) {
    DataSenderActive.first = true;
    DataSenderThreadId = Threader->SpawnThread((vtkThreadFunctionType)&DataSenderThread, this);
  }

  BroadcastStartTime = vtkPlusAccurateTimer::GetSystemTime();

  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
void* vtkPlusSimplePublisher::DataSenderThread(vtkMultiThreader::ThreadInfo* data) {
  vtkPlusSimplePublisher* self = static_cast<vtkPlusSimplePublisher*>((data->UserData));
  self->DataSenderActive.second = true;

  vtkPlusDevice* aDevice{nullptr};
  vtkPlusChannel* aChannel{nullptr};

  DeviceCollection aCollection;
  if (self->DataCollector->GetDevices(aCollection) != PLUS_SUCCESS || aCollection.size() == 0) {
    LOG_ERROR("Unable to retrieve devices. Check configuration and connection.");
    return NULL;
  }

  // Find the requested channel ID in all the devices
  for (auto& device : aCollection) {
    // TODO: we should not need this additional pointer.
    aDevice = device;
    if (aDevice->GetOutputChannelByName(aChannel, self->GetOutputChannelId()) == PLUS_SUCCESS) { break; }
  }

  if (aChannel == nullptr) {
    // The requested channel ID is not found
    if (!self->GetOutputChannelId().empty()) {
      // the user explicitly requested a specific channel, but none was found by that name
      // this is an error
      LOG_ERROR("Unable to start data sending. OutputChannelId not found: " << self->GetOutputChannelId());
      return nullptr;
    }
    // the user did not specify any channel, so just use the first channel that can be found in any device
    for (auto& device : aCollection) {
      // TODO: we should not need this additional pointer.
      aDevice = device;
      if (aDevice->OutputChannelCount() > 0) {
        aChannel = *(aDevice->GetOutputChannelsStart());
        break;
      }
    }
  }

  // If we didn't find any channel then return
  if (aChannel == nullptr) { LOG_WARNING("There are no channels to publish."); }

  self->BroadcastChannel = aChannel;
  if (self->BroadcastChannel) { self->BroadcastChannel->GetMostRecentTimestamp(self->LastSentTrackedFrameTimestamp); }

  double elapsedTimeSinceLastPacketSentSec = 0;
  while (self->DataSenderActive.first) {
    // Send image/tracking/string data
    SendLatestFrames(*self, elapsedTimeSinceLastPacketSentSec);
  }
  // Close thread
  self->DataSenderThreadId = -1;
  self->DataSenderActive.second = false;
  return nullptr;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusSimplePublisher::SendLatestFrames(vtkPlusSimplePublisher& self,
                                                    double& elapsedTimeSinceLastPacketSentSec) {
  vtkSmartPointer<vtkPlusTrackedFrameList> trackedFrameList = vtkSmartPointer<vtkPlusTrackedFrameList>::New();
  double startTimeSec{vtkPlusAccurateTimer::GetSystemTime()};

  // Acquire tracked frames since last acquisition (minimum 1 frame)
  if (self.LastProcessingTimePerFrameMs < 1) {
    // if processing was less than 1ms/frame then assume it was 1ms (1000FPS processing speed) to avoid division by zero
    self.LastProcessingTimePerFrameMs = 1;
  }
  int numberOfFramesToGet{std::max(self.MaxTimeSpentWithProcessingMs / self.LastProcessingTimePerFrameMs, 1)};
  // Maximize the number of frames to send
  numberOfFramesToGet = std::min(numberOfFramesToGet, 1);
  // We are always gonna get 1 frame, should we just change this?

  if (self.BroadcastChannel != nullptr) {
    if ((self.BroadcastChannel->HasVideoSource() && !self.BroadcastChannel->GetVideoDataAvailable()) ||
        (self.BroadcastChannel->ToolCount() > 0 && !self.BroadcastChannel->GetTrackingDataAvailable()) ||
        (self.BroadcastChannel->FieldCount() > 0 && !self.BroadcastChannel->GetFieldDataAvailable())) {
    } else {
      double oldestDataTimestamp{0.0};
      if (self.BroadcastChannel->GetOldestTimestamp(oldestDataTimestamp) == PLUS_SUCCESS) {
        if (self.LastSentTrackedFrameTimestamp < oldestDataTimestamp) {
          LOG_INFO("Simple broadcasting started. No data was available between "
                   << self.LastSentTrackedFrameTimestamp << "-" << oldestDataTimestamp
                   << "sec, therefore no data were broadcasted during this time period.");
          self.LastSentTrackedFrameTimestamp = oldestDataTimestamp + 0.1;
        }
        static vtkPlusLogHelper logHelper(60.0, 500000);
        CUSTOM_RETURN_WITH_FAIL_IF(
            self.BroadcastChannel->GetTrackedFrameList(self.LastSentTrackedFrameTimestamp, trackedFrameList,
                                                       numberOfFramesToGet) != PLUS_SUCCESS,
            "Failed to get tracked frame list from data collector (last recorded timestamp: "
                << std::fixed << self.LastSentTrackedFrameTimestamp);
      }
    }
  }

  // There is no new frame in the buffer
  if (trackedFrameList->GetNumberOfTrackedFrames() == 0) {
    vtkPlusAccurateTimer::Delay(0.005);
    elapsedTimeSinceLastPacketSentSec += vtkPlusAccurateTimer::GetSystemTime() - startTimeSec;

    // Send keep alive packet to clients
    if (elapsedTimeSinceLastPacketSentSec > 0.25) {
      elapsedTimeSinceLastPacketSentSec = 0;
      return PLUS_SUCCESS;
    }

    // TODO: this is all a bit useless, it might be enough to just send PLUS_FAIL.

    return PLUS_FAIL;
  }

  for (unsigned int i = 0; i < trackedFrameList->GetNumberOfTrackedFrames(); ++i) {
    // Send tracked frame
    self.SendTrackedFrame(*trackedFrameList->GetTrackedFrame(i));
    elapsedTimeSinceLastPacketSentSec = 0;
  }

  // Compute time spent with processing one frame in this round
  double computationTimeMs{(vtkPlusAccurateTimer::GetSystemTime() - startTimeSec) * 1000.0};

  // Update last processing time if new tracked frames have been acquired
  if (trackedFrameList->GetNumberOfTrackedFrames() > 0) {
    self.LastProcessingTimePerFrameMs = computationTimeMs / trackedFrameList->GetNumberOfTrackedFrames();
  }
  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusSimplePublisher::SendTrackedFrame(PlusTrackedFrame& trackedFrame) {
  int numberOfErrors{0};

  // Update transform repository with the tracked frame
  if (TransformRepository != nullptr) {
    if (TransformRepository->SetTransforms(trackedFrame) != PLUS_SUCCESS) {
      LOG_ERROR("Failed to set current transforms to transform repository");
      numberOfErrors++;
    }
  }

  // Convert relative timestamp to UTC
  double timestampSystem{trackedFrame.GetTimestamp()};  // save original timestamp, we'll restore it later
  double timestampUniversal{vtkPlusAccurateTimer::GetUniversalTimeFromSystemTime(timestampSystem)};
  trackedFrame.SetTimestamp(timestampUniversal);

  if (PublisherMessageType == "TRANSFORM") { /* TODO: Not currently implemented */
  } else if (PublisherMessageType == "IMAGE") {
    for (auto& imageStream : ImagesNames) {
      PlusTransformName imageTransformName = PlusTransformName(imageStream.Name, imageStream.EmbeddedTransformToFrame);
      vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();
      bool isValid{false};
      if (TransformRepository->GetTransform(imageTransformName, matrix.Get(), &isValid) != PLUS_SUCCESS) {
        LOG_WARNING("Failed to create " << PublisherMessageType << " message: cannot get image transform");
        numberOfErrors++;
        continue;
      }

      std::string deviceName{imageTransformName.From() + std::string("_") + imageTransformName.To()};
      if (trackedFrame.IsCustomFrameFieldDefined(PlusTrackedFrame::FIELD_FRIENDLY_DEVICE_NAME)) {
        // Allow overriding of device name with something human readable
        // The transform name is passed in the metadata
        deviceName = trackedFrame.GetCustomFrameField(PlusTrackedFrame::FIELD_FRIENDLY_DEVICE_NAME);
      }

      if (!trackedFrame.GetImageData()->IsImageValid()) {
        LOG_WARNING("Unable to send image message - image data is NOT valid!");
        return PLUS_FAIL;
      }

      simple_msgs::Header imageHeader;
      imageHeader.setSequenceNumber(imageStream.SequenceNumber);
      imageHeader.setFrameID(deviceName);
      imageHeader.setTimestamp(trackedFrame.GetTimestamp());

      simple_msgs::Image<uint8_t> imageMessage;
      imageMessage.setHeader(imageHeader);

      vtkImageData* frameImage = trackedFrame.GetImageData()->GetImage();

      int imageSizePixels[3]{0, 0, 0};
      double imageSpacingMm[3]{0, 0, 0};
      double imageOriginMm[3]{0, 0, 0};
      frameImage->GetDimensions(imageSizePixels);
      frameImage->GetSpacing(imageSpacingMm);
      frameImage->GetOrigin(imageOriginMm);

      imageMessage.setImageDimensions(imageSizePixels[0], imageSizePixels[1], imageSizePixels[2]);
      imageMessage.setImageResolution(imageSpacingMm[0], imageSpacingMm[1], imageSpacingMm[2]);
      imageMessage.setImageEncoding("TODO");

      double rotation_matrix[3][3];
      rotation_matrix[0][0] = matrix->GetElement(0, 0);
      rotation_matrix[0][1] = matrix->GetElement(0, 1);
      rotation_matrix[0][2] = matrix->GetElement(0, 2);
      rotation_matrix[1][0] = matrix->GetElement(1, 0);
      rotation_matrix[1][1] = matrix->GetElement(1, 1);
      rotation_matrix[1][2] = matrix->GetElement(1, 2);
      rotation_matrix[2][0] = matrix->GetElement(2, 0);
      rotation_matrix[2][1] = matrix->GetElement(2, 1);
      rotation_matrix[2][2] = matrix->GetElement(2, 2);

      vtkQuaternion<double> quaternion;
      quaternion.FromMatrix3x3(rotation_matrix);

      // Translation
      simple_msgs::Point image_position;
      image_position.setX(matrix->GetElement(0, 3));
      image_position.setY(matrix->GetElement(1, 3));
      image_position.setZ(matrix->GetElement(2, 3));

      // Rotation
      simple_msgs::Quaternion image_orientation;
      image_orientation.setX(quaternion.GetX());
      image_orientation.setY(quaternion.GetY());
      image_orientation.setZ(quaternion.GetZ());
      image_orientation.setW(quaternion.GetW());

      imageMessage.setOrigin({image_position, image_orientation});

      const uint8_t* vtkImagePointer = static_cast<const uint8_t*>(frameImage->GetScalarPointer());
      const int imageSize =
          imageSizePixels[0] * imageSizePixels[1] * imageSizePixels[2] * sizeof(uint8_t);  // TODO: check
      imageMessage.setImageData(vtkImagePointer, imageSize, 1);

      Publisher.publish(imageMessage);
      ++imageStream.SequenceNumber;
    }
  }

  // restore original timestamp
  trackedFrame.SetTimestamp(timestampSystem);

  return (numberOfErrors == 0 ? PLUS_SUCCESS : PLUS_FAIL);
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusSimplePublisher::Stop() {
  // Stop connection receiver thread
  if (DataSenderThreadId >= 0) {
    DataSenderActive.first = false;
    while (DataSenderActive.second) {
      // Wait until the thread stops
      vtkPlusAccurateTimer::DelayWithEventProcessing(0.2);
    }
    DataSenderThreadId = -1;
    LOG_DEBUG("ConnectionReceiverThread stopped");
  }

  LOG_INFO("Simple Publisher stopped.");

  return PLUS_SUCCESS;
}
