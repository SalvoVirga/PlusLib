#ifndef __VTKPLUSSIMPLEPUBLISHER_H
#define __VTKPLUSSIMPLEPUBLISHER_H

// Local includes
#include "vtkPlusDataCollector.h"
#include "vtkPlusServerExport.h"
#include "vtkPlusTransformRepository.h"

// VTK includes
#include <vtkMultiThreader.h>
#include <vtkObject.h>
#include <vtkSmartPointer.h>

// SIMPLE includes
#include <simple/publisher.hpp>

class vtkPlusServerExport vtkPlusSimplePublisher : public vtkObject {

public:

  struct ImageStream
  {
    ImageStream() = default;
    ImageStream(std::string n_1, std::string n_2) : Name(n_1), EmbeddedTransformToFrame(n_2) {}

    /*! Name of the image stream and the image message embedded transform "From" frame */
    std::string Name{""};
    /*! Name of the image message embedded transform "To" frame */
    std::string EmbeddedTransformToFrame{""};

    int SequenceNumber{0};
  };

  static vtkPlusSimplePublisher* New();
  /*! Configures and starts the server from the provided PlusSimplePublisher XML element */
  PlusStatus Start(vtkPlusDataCollector* dataCollector, vtkPlusTransformRepository* transformRepository, vtkXMLDataElement* serverElement, const std::string& configFilePath);

  /*! Stop the SIMPLE Publisher */
  PlusStatus Stop();

  /*! Read the configuration file in XML format and set up the devices */
  virtual PlusStatus ReadConfiguration(vtkXMLDataElement* serverElement, const std::string& aFilename);

  /*! Start publisher*/
  PlusStatus vtkPlusSimplePublisher::StartSimpleService();

  /*! Set server listening port */
  vtkSetMacro(ListeningPort, int);
  /*! Get server listening port */
  vtkGetMacroConst(ListeningPort, int);

  /*! Set data collector instance */
  vtkSetMacro(DataCollector, vtkPlusDataCollector*);
  vtkGetMacroConst(DataCollector, vtkPlusDataCollector*);

  /*! Set transform repository instance */
  vtkSetMacro(TransformRepository, vtkPlusTransformRepository*);
  vtkGetMacroConst(TransformRepository, vtkPlusTransformRepository*);

  /*! Set the required OutputChannelId */
  vtkSetStdStringMacro(OutputChannelId);
  /*! Get the required OutputChannelId */
  vtkGetStdStringMacro(OutputChannelId);

  vtkGetStdStringMacro(ConfigFilename);

  /*!
    Execute all commands in the queue from the current thread (useful if commands should be executed from the main thread)
    \return Number of executed commands
  */
  int ProcessPendingCommands();

protected:
  vtkPlusSimplePublisher();
  virtual ~vtkPlusSimplePublisher() = default;

private:

  /*! Simple publisher instance */
  simple::Publisher Publisher{};

  /*! Transform repository instance */
  vtkSmartPointer<vtkPlusTransformRepository> TransformRepository{nullptr};

  /*! Data collector instance */
  vtkSmartPointer<vtkPlusDataCollector> DataCollector{nullptr};

  /*! Multithreader instance for controlling threads */
  vtkSmartPointer<vtkMultiThreader> Threader{nullptr};

  /*! Server listening port */
  int ListeningPort{1};

  // Active flag for threads (first: request, second: respond )
  std::pair<bool, bool> DataSenderActive{};

  // Thread IDs
  int DataSenderThreadId{-1};

  /*! Last sent tracked frame timestamp */
  double LastSentTrackedFrameTimestamp{0.0};

  /*! Maximum time spent with processing (getting tracked frames, sending messages) per second (in milliseconds) */
  int MaxTimeSpentWithProcessingMs{50};

  /*! Time needed to process one frame in the latest recording round (in milliseconds) */
  int LastProcessingTimePerFrameMs{1.0};

  /*! Channel ID to request the data from */
  std::string OutputChannelId{""};

  /*! Channel to use for broadcasting */
  vtkPlusChannel* BroadcastChannel{nullptr};

  std::string ConfigFilename{""};

  std::string PublisherMessageType{""};
  std::vector<std::string> TransformNames{""};
  std::vector<ImageStream> ImagesNames{""};

  double BroadcastStartTime{0.0};
};

#endif // vtkPlusSimplePublisher_H
