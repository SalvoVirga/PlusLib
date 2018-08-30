#include "vtkPlusSimplePublisher.h"
#include <vtkMatrix3x3.h>
#include <vtkMatrix4x4.h>
#include <vtkQuaternion.h>
#include "vtkPlusTrackedFrameList.h"

vtkStandardNewMacro(vtkPlusSimplePublisher);

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
            TransformPublisher = simple::Publisher<simple_msgs::Pose>("tcp://*:" + std::to_string(ListeningPort));
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
          ImagePublisher = simple::Publisher<simple_msgs::Image<uint8_t>>("tcp://*:" + std::to_string(ListeningPort));
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

  // Before we spawn the thread, let's find the right channel to take our images from.
  DataSenderActive.second = true;

  DeviceCollection aCollection;
  if (DataCollector->GetDevices(aCollection) != PLUS_SUCCESS || aCollection.size() == 0) {
    LOG_ERROR("Unable to retrieve devices. Check configuration and connection.");
    return PLUS_FAIL;
  }

  // Find the requested channel ID in all the devices
  for (const auto& device : aCollection) {
    if (device->GetOutputChannelByName(BroadcastChannel, GetOutputChannelId()) == PLUS_SUCCESS) { break; }
  }

  if (BroadcastChannel == nullptr) {
    // The requested channel ID is not found
    if (!GetOutputChannelId().empty()) {
      // the user explicitly requested a specific channel, but none was found by that name
      // this is an error
      LOG_ERROR("Unable to start data sending. OutputChannelId not found: " << GetOutputChannelId());
      return PLUS_FAIL;
    }
    // The user did not specify any channel, so just use the first channel that can be found in any device.
    for (const auto& device : aCollection) {
      if (device->OutputChannelCount() > 0) {
        BroadcastChannel = *(device->GetOutputChannelsStart());
        break;
      }
    }
  }

  // If we didn't find any channel then return
  if (BroadcastChannel == nullptr) {
    LOG_ERROR("There are no valid channels to publish. Please specify a valid channel in your configuration file.");
    return PLUS_FAIL;
  }

  if (BroadcastChannel) { BroadcastChannel->GetMostRecentTimestamp(LastSentTrackedFrameTimestamp); }

  // If no thread was every spawned yet.
  if (!PublishThread.joinable()) {
    DataSenderActive.first = true;
    PublishThread = std::thread(&vtkPlusSimplePublisher::DataSenderThread, this);
  }

  BroadcastStartTime = vtkPlusAccurateTimer::GetSystemTime();

  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
void* vtkPlusSimplePublisher::DataSenderThread() {
  DataSenderActive.second = true;

  double elapsedTimeSinceLastPacketSentSec{0};
  while (DataSenderActive.first) {
    // Send image/tracking/string data
    SendLatestFrames(elapsedTimeSinceLastPacketSentSec);
  }
  // Close thread
  DataSenderThreadId = -1;
  DataSenderActive.second = false;
  return nullptr;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusSimplePublisher::SendLatestFrames(double elapsedTimeSinceLastPacketSentSec) {
  vtkSmartPointer<vtkPlusTrackedFrameList> trackedFrameList = vtkSmartPointer<vtkPlusTrackedFrameList>::New();
  double startTimeSec{vtkPlusAccurateTimer::GetSystemTime()};

  // Acquire tracked frames since last acquisition (minimum 1 frame)
  if (LastProcessingTimePerFrameMs < 1) {
    // if processing was less than 1ms/frame then assume it was 1ms (1000FPS processing speed) to avoid division by zero
    LastProcessingTimePerFrameMs = 1;
  }
  int numberOfFramesToGet{std::max(MaxTimeSpentWithProcessingMs / LastProcessingTimePerFrameMs, 1)};
  // Maximize the number of frames to send
  numberOfFramesToGet = std::min(numberOfFramesToGet, 100);

  if (BroadcastChannel != nullptr) {
    if ((BroadcastChannel->HasVideoSource() && !BroadcastChannel->GetVideoDataAvailable()) ||
        (BroadcastChannel->ToolCount() > 0 && !BroadcastChannel->GetTrackingDataAvailable()) ||
        (BroadcastChannel->FieldCount() > 0 && !BroadcastChannel->GetFieldDataAvailable())) {
    } else {
      double oldestDataTimestamp{0.0};
      if (BroadcastChannel->GetOldestTimestamp(oldestDataTimestamp) == PLUS_SUCCESS) {
        if (LastSentTrackedFrameTimestamp < oldestDataTimestamp) {
          LOG_INFO("Simple broadcasting started. No data was available between "
                   << LastSentTrackedFrameTimestamp << "-" << oldestDataTimestamp
                   << "sec, therefore no data were broadcasted during this time period.");
          LastSentTrackedFrameTimestamp = oldestDataTimestamp + 0.1;
        }
        static vtkPlusLogHelper logHelper(60.0, 500000);
        CUSTOM_RETURN_WITH_FAIL_IF(
            BroadcastChannel->GetTrackedFrameList(LastSentTrackedFrameTimestamp, trackedFrameList,
                                                  numberOfFramesToGet) != PLUS_SUCCESS,
            "Failed to get tracked frame list from data collector (last recorded timestamp: "
                << std::fixed << LastSentTrackedFrameTimestamp);
      }
    }
  }

  // There is no new frame in the buffer
  if (trackedFrameList->GetNumberOfTrackedFrames() == 0) { return PLUS_FAIL; }

  for (auto i = 0; i < trackedFrameList->GetNumberOfTrackedFrames(); ++i) {
    // Send tracked frame
    SendTrackedFrame(trackedFrameList->GetTrackedFrame(i));
    elapsedTimeSinceLastPacketSentSec = 0;
  }

  // Compute time spent with processing one frame in this round
  double computationTimeMs{(vtkPlusAccurateTimer::GetSystemTime() - startTimeSec) * 1000.0};

  // Update last processing time if new tracked frames have been acquired
  if (trackedFrameList->GetNumberOfTrackedFrames() > 0) {
    LastProcessingTimePerFrameMs = computationTimeMs / trackedFrameList->GetNumberOfTrackedFrames();
  }
  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusSimplePublisher::SendTrackedFrame(PlusTrackedFrame* trackedFrame) {
  int numberOfErrors{0};

  // Update transform repository with the tracked frame
  if (TransformRepository != nullptr) {
    if (TransformRepository->SetTransforms(*trackedFrame) != PLUS_SUCCESS) {
      LOG_ERROR("Failed to set current transforms to transform repository");
      numberOfErrors++;
    }
  }

  // Convert relative timestamp to UTC
  double timestampSystem{trackedFrame->GetTimestamp()};  // save original timestamp, we'll restore it later
  double timestampUniversal{vtkPlusAccurateTimer::GetUniversalTimeFromSystemTime(timestampSystem)};
  trackedFrame->SetTimestamp(timestampUniversal);

  if (PublisherMessageType == "TRANSFORM") {
    for (const auto& transformName : TransformNames) {
      vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();
      TransformRepository->GetTransform(transformName, matrix.Get());
      TransformPublisher.publish(vtkMatrixToSimplePose(matrix.Get()));
    }

  } else if (PublisherMessageType == "IMAGE") {
    for (auto& imageStream : ImagesNames) {
      PlusTransformName imageTransformName(imageStream.Name, imageStream.EmbeddedTransformToFrame);
      vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();
      bool isTransformValid{false};
      if (TransformRepository->GetTransform(imageTransformName, matrix.Get(), &isTransformValid) != PLUS_SUCCESS) {
        LOG_WARNING("Could not obtain the image transform " << imageTransformName << " the message type "
                                                            << PublisherMessageType
                                                            << " will be created without a transform.");
      }

      std::string deviceName{imageTransformName.From() + std::string("_") + imageTransformName.To()};
      if (trackedFrame->IsCustomFrameFieldDefined(PlusTrackedFrame::FIELD_FRIENDLY_DEVICE_NAME)) {
        // Allow overriding of device name with something human readable
        // The transform name is passed in the metadata
        deviceName = trackedFrame->GetCustomFrameField(PlusTrackedFrame::FIELD_FRIENDLY_DEVICE_NAME);
      }

      if (!trackedFrame->GetImageData()->IsImageValid()) {
        LOG_WARNING("Unable to send image message - image data is NOT valid!");
        return PLUS_FAIL;
      }

      simple_msgs::Header imageHeader;
      imageHeader.setSequenceNumber(imageStream.SequenceNumber);
      imageHeader.setFrameID(deviceName);
      imageHeader.setTimestamp(trackedFrame->GetTimestamp());

      simple_msgs::Image<uint8_t> imageMessage;
      imageMessage.setHeader(imageHeader);

      // Convert the vtkImageData to a simple_msgs::Image message.
      vtkImageData* frameImage = trackedFrame->GetImageData()->GetImage();

      int imageSizePixels[3]{0, 0, 0};
      double imageSpacingMm[3]{0, 0, 0};
      double imageOriginMm[3]{0, 0, 0};
      frameImage->GetDimensions(imageSizePixels);
      frameImage->GetSpacing(imageSpacingMm);
      frameImage->GetOrigin(imageOriginMm);

      imageMessage.setImageDimensions(imageSizePixels[0], imageSizePixels[1], imageSizePixels[2]);
      imageMessage.setImageSpacing(imageSpacingMm[0], imageSpacingMm[1], imageSpacingMm[2]);
      imageMessage.setImageEncoding("");  // TODO

      if (isTransformValid) { imageMessage.setOrigin(vtkMatrixToSimplePose(matrix.Get())); }

      const uint8_t* vtkImagePointer = static_cast<const uint8_t*>(frameImage->GetScalarPointer());
      const int imageSize = imageSizePixels[0] * imageSizePixels[1] * imageSizePixels[2] * sizeof(uint8_t);
      imageMessage.setImageData(vtkImagePointer, imageSize, 1);

      ImagePublisher.publish(imageMessage);
      ++imageStream.SequenceNumber;
    }
  }

  // restore original timestamp
  trackedFrame->SetTimestamp(timestampSystem);

  return (numberOfErrors == 0 ? PLUS_SUCCESS : PLUS_FAIL);
}

//----------------------------------------------------------------------------
simple_msgs::Pose vtkPlusSimplePublisher::vtkMatrixToSimplePose(vtkMatrix4x4* matrix) {
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

  return {image_position, image_orientation};
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
