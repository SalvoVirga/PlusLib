/*=Plus=header=begin======================================================
  Program: Plus
  Copyright (c) Laboratory for Percutaneous Surgery. All rights reserved.
  See License.txt for details.
=========================================================Plus=header=end*/

#include "PlusConfigure.h"

#include "vtksys/CommandLineArguments.hxx"
#include "vtkSmartPointer.h"
#include "vtkTransform.h"
#include "vtkMatrix4x4.h"

#include "PlusMath.h"
#include "vtkTransformRepository.h"

int main(int argc, char **argv)
{
  // Parse command-line arguments
	bool printHelp(false);
	int verboseLevel(vtkPlusLogger::LOG_LEVEL_DEFAULT);
	vtksys::CommandLineArguments args;
	args.Initialize(argc, argv);
	args.AddArgument("--help", vtksys::CommandLineArguments::NO_ARGUMENT, &printHelp, "Print this help.");	
	args.AddArgument("--verbose", vtksys::CommandLineArguments::EQUAL_ARGUMENT, &verboseLevel, "Verbose level (1=error only, 2=warning, 3=info, 4=debug, 5=trace)");	
	if ( !args.Parse() )
	{
		std::cerr << "Problem parsing arguments" << std::endl;
		std::cout << "Help: " << args.GetHelp() << std::endl;
		exit(EXIT_FAILURE);
	}
	if ( printHelp ) 
	{
		std::cout << "Help: " << args.GetHelp() << std::endl;
		exit(EXIT_SUCCESS); 

	}
	vtkPlusLogger::Instance()->SetLogLevel(verboseLevel);

  /////////////////////////////////////////////////////////////////////////////
  // Set up coordinate transforms

  vtkSmartPointer<vtkTransformRepository> transformRepository=vtkSmartPointer<vtkTransformRepository>::New();

  vtkSmartPointer<vtkMatrix4x4> mxProbeToTracker=vtkSmartPointer<vtkMatrix4x4>::New();  
  mxProbeToTracker->Element[0][3]=15;
  mxProbeToTracker->Element[0][0]=-0.5;
  mxProbeToTracker->Element[1][1]=-0.8;
  transformRepository->SetTransform("Probe", "Tracker", mxProbeToTracker, vtkTransformRepository::TRANSFORM_INVALID);

  vtkSmartPointer<vtkMatrix4x4> mxStylusToTracker=vtkSmartPointer<vtkMatrix4x4>::New();  
  mxStylusToTracker->Element[1][3]=25;
  mxStylusToTracker->Element[0][0]=0.1;
  mxStylusToTracker->Element[1][0]=0.2;
  mxStylusToTracker->Element[2][0]=-0.4;
  transformRepository->SetTransform("Stylus", "Tracker", mxStylusToTracker);

  vtkSmartPointer<vtkMatrix4x4> mxPhantomToTracker=vtkSmartPointer<vtkMatrix4x4>::New();    
  mxPhantomToTracker->Element[2][2]=-4;
  mxPhantomToTracker->Element[0][3]=2;
  mxPhantomToTracker->Element[1][3]=2;
  mxPhantomToTracker->Element[2][3]=20;
  transformRepository->SetTransform("Phantom", "Tracker", mxPhantomToTracker);

  vtkSmartPointer<vtkMatrix4x4> mxStylusTipToStylus=vtkSmartPointer<vtkMatrix4x4>::New();    
  mxStylusTipToStylus->Element[2][3]=4;
  mxStylusTipToStylus->Element[1][1]=0.2;
  mxStylusTipToStylus->Element[2][0]=-0.2;
  mxStylusTipToStylus->Element[2][1]=0.4;
  mxStylusTipToStylus->Element[2][2]=-0.2;
  transformRepository->SetTransform("StylusTip", "Stylus", mxStylusTipToStylus);
  
  transformRepository->PrintSelf(std::cout, vtkIndent() ); 

  /////////////////////////////////////////////////////////////////////////////
  // Test StylusTipToTracker computation
  // Compute with transform Repository 
  vtkSmartPointer<vtkMatrix4x4> mxStylusTipToTracker=vtkSmartPointer<vtkMatrix4x4>::New();    
  vtkTransformRepository::TransformStatus status=vtkTransformRepository::TRANSFORM_INVALID;
  transformRepository->GetTransform("StylusTip", "Tracker", mxStylusTipToTracker, &status);
  LOG_INFO("StylusTipToTracker (computed by transformRepository):");
  mxStylusTipToTracker->PrintSelf(std::cout, vtkIndent());
  // Compute manually
  vtkSmartPointer<vtkTransform> transformStylusTipToTrackerManual=vtkSmartPointer<vtkTransform>::New();
  transformStylusTipToTrackerManual->Concatenate(mxStylusToTracker);
  transformStylusTipToTrackerManual->Concatenate(mxStylusTipToStylus);
  vtkMatrix4x4* mxStylusTipToTrackerManual=transformStylusTipToTrackerManual->GetMatrix();
  LOG_INFO("StylusTipToTracker (computed manually):");
  mxStylusTipToTrackerManual->PrintSelf(std::cout, vtkIndent());
  // Compare
  double posDiff=PlusMath::GetPositionDifference(mxStylusTipToTracker, mxStylusTipToTrackerManual); 
  double orientDiff=PlusMath::GetOrientationDifference(mxStylusTipToTracker, mxStylusTipToTrackerManual); 
  LOG_INFO("Position difference: "<< posDiff);
  LOG_INFO("Orientation difference: "<< orientDiff);
  if (fabs(posDiff)>0.001 || fabs(orientDiff)>0.001)
  {
    LOG_ERROR("Mismatch between transforms computed by transformRepository and manually");
    return EXIT_FAILURE;
  }
  if (status!=vtkTransformRepository::TRANSFORM_VALID)
  {
    LOG_ERROR("StylusTipToTracker should be valid");
    return EXIT_FAILURE;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Test PhantomToStylusTip computation
  // Compute with transform Repository 
  vtkSmartPointer<vtkMatrix4x4> mxPhantomToStylusTip=vtkSmartPointer<vtkMatrix4x4>::New();    
  status=vtkTransformRepository::TRANSFORM_INVALID;
  transformRepository->GetTransform("Phantom", "StylusTip", mxPhantomToStylusTip, &status);
  LOG_INFO("PhantomToStylusTip (computed by transformRepository):");
  mxPhantomToStylusTip->PrintSelf(std::cout, vtkIndent());
  // Compute manually
  vtkSmartPointer<vtkTransform> transformPhantomToStylusTipManual=vtkSmartPointer<vtkTransform>::New();
  vtkSmartPointer<vtkMatrix4x4> mxStylusToStylusTip=vtkSmartPointer<vtkMatrix4x4>::New();    
  vtkMatrix4x4::Invert(mxStylusTipToStylus, mxStylusToStylusTip);
  transformPhantomToStylusTipManual->Concatenate(mxStylusToStylusTip);
  vtkSmartPointer<vtkMatrix4x4> mxTrackerToStylus=vtkSmartPointer<vtkMatrix4x4>::New();    
  vtkMatrix4x4::Invert(mxStylusToTracker, mxTrackerToStylus);
  transformPhantomToStylusTipManual->Concatenate(mxTrackerToStylus);
  transformPhantomToStylusTipManual->Concatenate(mxPhantomToTracker);  
  vtkMatrix4x4* mxPhantomToStylusTipManual=transformPhantomToStylusTipManual->GetMatrix();
  LOG_INFO("PhantomToStylusTip (computed manually):");
  mxPhantomToStylusTipManual->PrintSelf(std::cout, vtkIndent());
  // Compare
  posDiff=PlusMath::GetPositionDifference(mxPhantomToStylusTip, mxPhantomToStylusTipManual); 
  orientDiff=PlusMath::GetOrientationDifference(mxPhantomToStylusTip, mxPhantomToStylusTipManual); 
  LOG_INFO("Position difference: "<< posDiff);
  LOG_INFO("Orientation difference: "<< orientDiff);
  if (fabs(posDiff)>0.001 || fabs(orientDiff)>0.001)
  {
    LOG_ERROR("Mismatch between transforms computed by transformRepository and manually");
    return EXIT_FAILURE;
  }
  if (status!=vtkTransformRepository::TRANSFORM_VALID)
  {
    LOG_ERROR("PhantomToStylusTip should be valid");
    return EXIT_FAILURE;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Test PhantomToStylusTip computation after update of an existing transform
  // Update transform
  mxStylusToTracker->Element[0][0]=0.9;
  transformRepository->SetTransform("Stylus", "Tracker", mxStylusToTracker);
  // Compute with transform Repository 
  transformRepository->GetTransform("Phantom", "StylusTip", mxPhantomToStylusTip, &status);
  LOG_INFO("PhantomToStylusTip (computed by transformRepository):");
  mxPhantomToStylusTip->PrintSelf(std::cout, vtkIndent());
  // Compute manually
  transformPhantomToStylusTipManual->Identity();
  transformPhantomToStylusTipManual->Concatenate(mxStylusToStylusTip);  
  vtkMatrix4x4::Invert(mxStylusToTracker, mxTrackerToStylus);
  transformPhantomToStylusTipManual->Concatenate(mxTrackerToStylus);
  transformPhantomToStylusTipManual->Concatenate(mxPhantomToTracker);  
  mxPhantomToStylusTipManual=transformPhantomToStylusTipManual->GetMatrix();
  LOG_INFO("PhantomToStylusTip (computed manually):");
  mxPhantomToStylusTipManual->PrintSelf(std::cout, vtkIndent());
  // Compare
  posDiff=PlusMath::GetPositionDifference(mxPhantomToStylusTip, mxPhantomToStylusTipManual); 
  orientDiff=PlusMath::GetOrientationDifference(mxPhantomToStylusTip, mxPhantomToStylusTipManual); 
  LOG_INFO("Position difference: "<< posDiff);
  LOG_INFO("Orientation difference: "<< orientDiff);
  if (fabs(posDiff)>0.001 || fabs(orientDiff)>0.001)
  {
    LOG_ERROR("Mismatch between transforms computed by transformRepository and manually");
    return EXIT_FAILURE;
  }
  if (status!=vtkTransformRepository::TRANSFORM_VALID)
  {
    LOG_ERROR("PhantomToStylusTip should be valid");
    return EXIT_FAILURE;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Check if invalid transform flag is correctly propagated
  if (transformRepository->GetTransformStatus("Probe", "Stylus", status)!=PLUS_SUCCESS)
  {
    LOG_ERROR("Cannot get ProbeToStylus transform status");
    return EXIT_FAILURE;
  }
  if (status!=vtkTransformRepository::TRANSFORM_INVALID)
  {
    LOG_ERROR("The ProbeToStylus transform should be invalid");
    return EXIT_FAILURE;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Check if non-existing transforms are handled properly
  if (transformRepository->GetTransformStatus("Probe", "StylusNonExisting", status)==PLUS_SUCCESS)
  {
    LOG_ERROR("A non-existing transform has been reported to be found");
    return EXIT_FAILURE;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Check circle detection
  vtkSmartPointer<vtkMatrix4x4> mxProbeToPhantom=vtkSmartPointer<vtkMatrix4x4>::New();    
  if (transformRepository->SetTransform("Probe", "Phantom", mxProbeToPhantom)==PLUS_SUCCESS)
  {
    LOG_ERROR("Circular reference between transforms is not detected");
    return EXIT_FAILURE;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Check delete
  if (transformRepository->DeleteTransform("Tracker", "Probe")==PLUS_SUCCESS)
  {
    LOG_ERROR("Only the inverse of the transform has been set, delete should not have been allowed");
    return EXIT_FAILURE;
  }
  if (transformRepository->DeleteTransform("Probe", "Tracker")!=PLUS_SUCCESS)
  {
    LOG_ERROR("Transform delete failed");
    return EXIT_FAILURE;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Check circle detection - after delete
  if (transformRepository->SetTransform("Probe", "Phantom", mxProbeToPhantom)!=PLUS_SUCCESS)
  {
    LOG_ERROR("Set transform should have been succeeded");
    return EXIT_FAILURE;
  }

  /////////////////////////////////////////////////////////////////////////////
  // Check clear
  transformRepository->Clear();
  if (transformRepository->GetTransform("StylusTip", "Tracker", mxStylusTipToTracker, &status)==PLUS_SUCCESS)
  {
    LOG_ERROR("GetTransform should have failed after clearing the transform repository");
    return EXIT_FAILURE;
  }


  LOG_INFO("Test successfully completed");
	return EXIT_SUCCESS; 
 }