#ifndef __VTKPLUSSIMPLEPUBLISHER_H
#define __VTKPLUSSIMPLEPUBLISHER_H

// System includes
#include <thread>
#include <atomic>

// Local includes
#include "vtkPlusDataCollector.h"
#include "vtkPlusServerExport.h"
#include "vtkPlusTransformRepository.h"

// VTK includes
#include <vtkObject.h>

// SIMPLE includes
#include "simple/publisher.hpp"
#include "simple_msgs/image.hpp"

class vtkPlusServerExport vtkPlusSimplePublisher {
public:
  struct ImageStream {
    ImageStream() = default;
	ImageStream(const std::string& n_1, const std::string& n_2) : Name{ n_1 }, EmbeddedTransformToFrame{ n_2 } {}

    /*! Name of the image stream and the image message embedded transform "From" frame */
    std::string Name{""};
    /*! Name of the image message embedded transform "To" frame */
    std::string EmbeddedTransformToFrame{""};
    int SequenceNumber{0};
  };

  vtkPlusSimplePublisher() = default;

  virtual ~vtkPlusSimplePublisher() {
	  if (PublishThread.joinable()) { PublishThread.join(); }
  };

  //vtkPlusSimplePublisher(const vtkPlusSimplePublisher&) {};

  /*! Configures and starts the server from the provided PlusSimplePublisher XML element */
  PlusStatus Start(vtkPlusDataCollector* dataCollector, vtkPlusTransformRepository* transformRepository,
                   vtkXMLDataElement* serverElement, const std::string& configFilePath);

  /*! Stop the SIMPLE Publisher */
  PlusStatus Stop();

  /*! Read the configuration file in XML format and set up the devices */
  virtual PlusStatus ReadConfiguration(vtkXMLDataElement* serverElement, const std::string& aFilename);



private:
  /*! Start publisher*/
  PlusStatus StartSimpleService();

  /*! Thread for sending data to clients */
  void DataSenderThread();

  /*! Attempt to send any unsent frames to clients, if unsuccessful, accumulate an elapsed time */
  PlusStatus SendLatestFrames(double elapsedTimeSinceLastPacketSentSec);

  /*! Tracked frame interface, sends the selected message type and data to all clients */
  virtual PlusStatus SendTrackedFrame(PlusTrackedFrame* trackedFrame);

  simple_msgs::Pose vtkMatrixToSimplePose(vtkMatrix4x4* matrix);


  /*** Member variables ***/
  /*! Publish Thread */
  std::thread PublishThread{};

  /*! Transform repository instance */
  vtkPlusTransformRepository* TransformRepository{ nullptr };

  /*! Data collector instance */
  vtkPlusDataCollector* DataCollector{ nullptr };

  /*! Channel to use for broadcasting */
  vtkPlusChannel* BroadcastChannel{ nullptr };

  /*! Server listening port */
  int ListeningPort{1};

  // Active flag for threads (first: request, second: respond )
  std::atomic<bool> DataSenderActive{false};

  /*! Last sent tracked frame timestamp */
  double LastSentTrackedFrameTimestamp{0.0};

  /*! Maximum time spent with processing (getting tracked frames, sending messages) per second (in milliseconds) */
  int MaxTimeSpentWithProcessingMs{50};

  /*! Time needed to process one frame in the latest recording round (in milliseconds) */
  int LastProcessingTimePerFrameMs{1};

  /*! Channel ID to request the data from */
  std::string OutputChannelId{""};

  std::string ConfigFilename{""};
  std::string PublisherMessageType{""};
  std::vector<std::string> TransformNames{};
  std::vector<ImageStream> ImagesNames{};

  /*! Simple publisher instance */
  simple::Publisher<simple_msgs::Image<uint8_t>> ImagePublisher;
  simple::Publisher<simple_msgs::Pose> TransformPublisher;

  // Custom function to read an int from the xml tree - PLUS uses some damn old macros....
  PlusStatus xml_read_scalar_attribute_required(std::string xml_attribute, vtkXMLDataElement* xml_elem, int& variable)
  { 
    int tmpValue = 0; 
    if (xml_elem->GetScalarAttribute(xml_attribute.c_str(), tmpValue) )
    { 
		variable = tmpValue;
		return PLUS_SUCCESS;
    } 
    else  
    { 
      LOG_ERROR("Unable to find required " << xml_attribute << " attribute in " << (xml_elem->GetName() ? xml_elem->GetName() : "(undefined)") << " element in device set configuration");
      return PLUS_FAIL; 
    } 
  }

  // Custom function to read a string from the xml tree - PLUS uses some damn old macros....
  PlusStatus xml_read_string_attribute_required(std::string xml_attribute, vtkXMLDataElement* xml_elem, std::string& variable)
  {
	const char* destinationXmlElementVar = xml_elem->GetAttribute(xml_attribute.c_str());
	if (destinationXmlElementVar == nullptr)  
	{ 
	  LOG_ERROR("Unable to find required " << xml_attribute << " attribute in " << (xml_elem->GetName() ? xml_elem->GetName() : "(undefined)") << " element in device set configuration");
	  return PLUS_FAIL; 
     } 
	variable = std::string(destinationXmlElementVar);		
	return PLUS_SUCCESS;
  }
};

#endif  // vtkPlusSimplePublisher_H
