/*****************************************************************************
* ia_callback.cpp
*****************************************************************************
*
* Project:      HALCON/C++
* Description:  Example program for HALCON/C++
*
* (c) 2011-2014 by MVTec Software GmbH
*               www.mvtec.com
*****************************************************************************
*
* Shows the usage of the HALCON Image Acquisition callback functionality
*
****************************************************************************/

#include "ros/ros.h"


#ifndef __APPLE__
#  include "halconcpp/HalconCpp.h"
#else
#  ifndef HC_LARGE_IMAGES
#    include <HALCONCpp/HalconCpp.h>
#  else
#    include <HALCONCppxl/HalconCpp.h>
#  endif
#endif

#ifdef _WIN32
#include <windows.h>
#include <process.h>
#else
#include <pthread.h>
#endif

#include <string.h>

using namespace HalconCpp;
using namespace std;
using namespace ros;

// Define Prototypes
HTuple fileSelection(char c);
int  showAvailableDevices(char***);
bool initDevice(char**,HFramegrabber&,int);
int  showAvailableCallbackTypes(char***,HFramegrabber&);
bool registerCallbackType(HFramegrabber&,char**,int, int&);
int  readUserSelection(int);
bool exitProgram(HFramegrabber&,char**,char**,int, int, int, int);
void workerThread(void *);
void __stdcall MyCallbackFunction(void *acq, void *context, void *user_context);


// Device specific information, e.g. GigEVision
// Please check the documentation to figure out which HALCON Image Acquisition
// Interface supports the callback functionality
const char* cInterface = "Video4Linux2"; // "1394IIDC", "ADLINK", "BitFlow",
// "GenICamTL", "LinX", "MILLite",
// "MultiCam", "pylon", "SaperaLT" ...

// In some cases you have to adapt the next lines to open your desired device
const char* cCameraType = "default"; // e.g. Path to camera configuration
// file

// Used structures for synchronisation and program settings
#ifdef _WIN32
HANDLE     gThreadHandle;
#else
pthread_t  gThreadHandle;
#endif
HCondition *gAcqCondition;
HMutex     *gAcqMutex;
HBOOL      gThreadRunning;
HBOOL      gCountCallbacks=0;
const INT  cMaxGrabImage=10;


HTuple fileSelection(char c, float &unit)
{
    cout << "Please select a file:" << endl;
    cout << "1. bearing \n"
         << "2. bluebox \n"
         << "3. flange screw \n"
         << "4. mechanicalpipe\n"
         << "5. mechanicaltree \n"
         << "6. ring wheel \n"
         << "7. screw m5x12 - 8.8 \n"
         << "8. spacer \n"
         << "9. reduction-150-120-fillet-5\n"
         << "10. simplifiedBearing \n"
         << "11. whole part "
         <<endl;

    int fileIndex;
    cin >> fileIndex;

    while(fileIndex<1 || fileIndex > 11){
        cout << "Wrong input! Select again!" << endl;
        cin >> fileIndex;
    };

    HTuple fileName;
    if(fileIndex == 1)
        fileName = HTuple("bearing");
    else if(fileIndex == 2)
        fileName = HTuple("bluebox");
    else if(fileIndex == 3)
        fileName = HTuple("flange screw");
    else if(fileIndex == 4)
        fileName = HTuple("mechanicalpipe");
    else if(fileIndex == 5)
        fileName = HTuple("mechanicaltree");
    else if(fileIndex == 6)
        fileName = HTuple("ring wheel");
    else if(fileIndex == 7)
        fileName = HTuple("screw m5x12 - 8.8");
    else if(fileIndex == 8)
        fileName = HTuple("spacer");
    else if (fileIndex == 9)
        fileName = HTuple("reduction-150-120-fillet-5");
    else if (fileIndex == 10)
        fileName = HTuple("simplifiedBearing");
    else
        fileName = HTuple("whole part");

    unit = 1;
    if (fileIndex == 2 ||fileIndex == 10)
        unit = 0.001;


    return fileName;

}


// Define a user-specific callback function
void __stdcall MyCallbackFunction(void *acq, void *context, void *user_context)
{
    cout << "AcqHandle: "  << (int*)acq << " is called by  UserContext: "
         << (char*)user_context << '\n';

    // Wake up the workerThread, to do the specific calculations.
    // In this case, the worker thread is only called, if the given
    // user context was previously set to the corresponding user
    // context 'ExposureEnd' or 'transfer_end'.
    if (!strcmp ((char*)user_context,"ExposureEnd") ||
            !strcmp ((char*)user_context,"transfer_end"))
    {
        gAcqMutex->LockMutex();
        gAcqCondition->SignalCondition();
        gCountCallbacks++;
        gAcqMutex->UnlockMutex();
    }
}

bool get_det_obj_service_callback(){

}


// Thread function to handle the callback type specific calculations.
// Please note, that this thread is not synchronized. This acquisition
// handle is the same handle as requested inside the initDevice() function.
// If you change the device configuration by using this handle, any further
// existing device handle will be affected and vice versa.
void workerThread(void *parameters)
{
    ros::NodeHandle nh;
    ros::ServiceServer get_det_obj;
    //get_det_obj = nh.advertiseService("get_det_obj", &get_det_obj_service_callback);

    HFramegrabber *acq = (HFramegrabber*)parameters;
    HImage image;
    HTuple *status = new HTuple();

    HTuple *object_model = new HTuple();

    image = acq->GrabImageAsync(-1);

    //halcon_install/doc/html/reference/operators/calibrate_cameras.html
    float *camParamArray = (float*)malloc(sizeof(float)*8);
    //focus: 1551 pixels
    camParamArray[0] = 0.0198528; //focus
    camParamArray[1] = 0; //Kappa
    camParamArray[2] = 1.28e-005; //Sx
    camParamArray[3] = 1.28e-005; //Sy
    camParamArray[4] = 611,629865; //Cx
    camParamArray[5] = 369,303181; //Cy
    camParamArray[6] = image.Width(); //ImageWidth
    camParamArray[7] = image.Height(); //ImageHeight
    HTuple *camParams = new HTuple(camParamArray, 8);

    HTuple *genParam = new HTuple();
    HTuple *genParamValue = new HTuple();

    HTuple start, end, time1;

    cout << "Create a new model?(Y/N)" << endl;
    char c;
    cin >> c;


    while(c == 'Y' || c == 'y'){
        float unit;
        HTuple fileName = fileSelection(c, unit);


        ROS_INFO("Preparing models...");
        try{
            ReadObjectModel3d(fileName, unit, *genParam, *genParamValue, object_model, status);
        }
        catch(HException &exc)
        {
            cout << exc.ErrorMessage() << endl;
        }

        std::cout << "status: " << status->TupleSelect(0).ToString() << std::endl;
        std::cout << object_model->Length() << std::endl;

        PrepareObjectModel3d(*object_model, "shape_based_matching_3d", "true", *genParam, *genParamValue);
        std::cout << object_model->Length() << std::endl;
        HTuple *genParam1 = new HTuple("num_levels");
        HTuple *genParamValue1 = new HTuple(1);

        ROS_INFO("Creating 3D Model...");
        HTuple *shape_model = new HTuple();
        try{
            //CreateShapeModel3d(*object_model, *camParams, 0, 0, 0, "gba", -0.17, 0.17, -0.17, 0.17, -0.17, 0.17, 0.3, 0.4, 10, *genParam1, *genParamValue1, shape_model);
            CountSeconds(&start);
            //        CreateShapeModel3d(*object_model, *camParams, 0, 0, 0, "gba", -(HTuple(55).TupleRad()),
            //                           HTuple(55).TupleRad(), -(HTuple(55).TupleRad()), HTuple(55).TupleRad(), 0,
            //                           HTuple(360).TupleRad(), 0.3, 0.31, 10, (HTuple("lowest_model_level").Append("metric").Append("union_adjacent_contours")),
            //                           (HTuple(2).Append("ignore_part_polarity").Append("true")), shape_model);
            //            CreateShapeModel3d(*object_model, *camParams, 0, 0, 0, "gba", -(HTuple(55).TupleRad()),
            //                               HTuple(55).TupleRad(), -(HTuple(55).TupleRad()), HTuple(55).TupleRad(), 0,
            //                               HTuple(360).TupleRad(), 0.07, 0.08, 10, (HTuple("lowest_model_level").Append("metric")),
            //                               (HTuple(2).Append("ignore_part_polarity")), shape_model);

            CreateShapeModel3d(*object_model, *camParams, 0, 0, 0, "gba",
                               -(HTuple(55).TupleRad()),HTuple(55).TupleRad(),
                               -(HTuple(55).TupleRad()), HTuple(55).TupleRad(),
                               -HTuple(0).TupleRad(), HTuple(360).TupleRad(),
                               1.11, 1.12, 10,
                               (HTuple("lowest_model_level").Append("metric").Append("union_adjacent_contours")),
                               (HTuple(2).Append("ignore_part_polarity").Append("true")),
                               shape_model);

            CountSeconds(&end);
            time1 = end - start;
            cout << "Model creation takes " << time1.TupleString(".1f").ToString() << " seconds." << endl;
        }
        catch(HException &exc)
        {
            cout << exc.ErrorMessage() << endl;
        }
        std::cout << shape_model->Length() << std::endl;
        ROS_INFO("Writing shape model...");
        WriteShapeModel3d(*shape_model, fileName+".sm3");

        cout << "Create another new model?(Y/N)" << endl;
        cin >> c;

    }

    HTuple shape_model[6];
    cout << "Which model do you want to load?" << endl;
    c = 'y';
    int modelNumber = 0;
    while(c == 'Y' || c == 'y'){
        float unit;
        HTuple fileName = fileSelection(c, unit);

        cout << "Loading selected model..." << modelNumber<< endl;

        ReadShapeModel3d(fileName+".sm3", &(shape_model[modelNumber]));
        modelNumber ++;

        cout << "(" << modelNumber << " /6) models have been loaded." << endl;
        cout << "Load another model?(Y/N)" << endl;
        cin >> c;
    }
    gAcqMutex->LockMutex();

    //create visulization window:
    //    HSystem::SetWindowAttr("border_width",HTuple(0));

    //  HWindow	window(16,16,image.Width(),image.Height(),0,"visible","");

    //  window.DispObj(image);

    HTuple hv_WindowHandle;
    if (HDevWindowStack::IsOpen())
        CloseWindow(HDevWindowStack::Pop());
    SetWindowAttr("background_color","black");
    OpenWindow(0,0,image.Width(),image.Height(),0, "","",&hv_WindowHandle);
    HDevWindowStack::Push(hv_WindowHandle);
    if (HDevWindowStack::IsOpen())
        SetLineWidth(HDevWindowStack::GetActive(),2);

    HTuple  hv_MatchingParameters, hv_MatchingParameterValues;
    hv_MatchingParameters.Clear();
    hv_MatchingParameters[0] = "num_matches";
    //hv_MatchingParameters[1] = "pose_refinement";
    hv_MatchingParameters[1] = "max_overlap";
    hv_MatchingParameterValues.Clear();
    hv_MatchingParameterValues[0] = 5;
    //hv_MatchingParameterValues[1] = "least_squares_high";
    hv_MatchingParameterValues[1] = 0;

    HTuple hv_Colors;
    hv_Colors.Clear();
    hv_Colors[0] = "blue";
    hv_Colors[1] = "green";
    hv_Colors[2] = "red";
    hv_Colors[3] = "yellow";
    hv_Colors[4] = "khaki";
    hv_Colors[5] = "aquamarine";

    for(int iia = 0; iia < 1000; iia++)
    {
        // Enter the WaitCondition function with a locked mutex to avoid
        // problems with the image acquisition handle. The WaitCondition
        // function will always exit with a locked mutex handle.
        //    gAcqCondition->WaitCondition(*gAcqMutex);

        if(!gThreadRunning || gCountCallbacks > cMaxGrabImage)
            break;

        cout << "Do the thread specific calculations\n" ;
        //image = acq->GrabImageAsync(-1);
        try
        {
            image = acq->GrabImageAsync(-1);
        }
        catch(HException &exc)
        {
            cout << exc.ErrorMessage() << endl;
        }

        // window.DispObj(image);



        HImage greyImage;
        Rgb1ToGray(image, &greyImage);



        //        if (HDevWindowStack::IsOpen())
        //          SetColor(HDevWindowStack::GetActive(),HTuple(hv_Colors[0]));
        //        if (HDevWindowStack::IsOpen())
        //          DispObj(greyImage, HDevWindowStack::GetActive());

        HTuple *Pose = new HTuple();
        HTuple *CovPose = new HTuple();
        HTuple *Score = new HTuple();
        //      GrabImageAsync(image, acq, -1);
        //      std::cout << image.Width() << "," << image.Height() << std::endl;
        ROS_INFO("Searching for object...");
        CountSeconds(&start);
        for(int modelIndex = 0; modelIndex < modelNumber; modelIndex++){
            try{
                //FindShapeModel3d(greyImage, shape_model[0], 0.65, 0.9, 0, "recompute_score", "true", Pose, CovPose, Score);
                FindShapeModel3d(greyImage, shape_model[modelIndex], 0.65, 0.9, 0,
                                 hv_MatchingParameters, hv_MatchingParameterValues,
                                 Pose, CovPose, Score);
                CountSeconds(&end);
                time1 = end - start;
                cout << "Find object in " << time1.TupleString(".1f").ToString() << " seconds." << endl;
            }
            catch(HException &exc)
            {
                cout << exc.ErrorMessage() << endl;
            }
            if (modelIndex == 0 && HDevWindowStack::IsOpen())
                DispObj(greyImage, HDevWindowStack::GetActive());
            if(Score->TupleLength()>0){
                std::cout << Score->TupleLength().ToString() << " matches have been found!" << std::endl;
                for(int i = 0; i < Score->TupleLength(); i++){
                    std::cout << Score->TupleSelect(i).ToString() << "  ";
                }
                std::cout << endl;

                //Visualize the found matches in the image by
                //projecting the 3D shape model with the pose of the match
                HObject ho_ModelContours;
                HTuple end_val58 = (Score->TupleLength())-1;
                HTuple step_val58 = 1;
                SetLineStyle(hv_WindowHandle, (HTuple(10).Append(10)));
                for (HTuple hv_J=0; hv_J.Continue(end_val58, step_val58); hv_J += step_val58)
                {
                    HTuple hv_PoseTmp = Pose->TupleSelectRange(hv_J*7,(hv_J*7)+6);
                    ProjectShapeModel3d(&ho_ModelContours, shape_model[modelIndex], *camParams, hv_PoseTmp,
                                        "true", HTuple(30).TupleRad());
                    if (HDevWindowStack::IsOpen()){
                        ROS_INFO("Displaying object...");
                        if (HDevWindowStack::IsOpen())
                            SetColor(HDevWindowStack::GetActive(),HTuple(hv_Colors[modelIndex]));
                        DispObj(ho_ModelContours, HDevWindowStack::GetActive());
                    }
                }
            }
        }
    }
    gAcqMutex->UnlockMutex();
    cout << "End of acquisition thread\n";
}


int main(int argc, char *argv[])
{
    SetSystem(HTuple("opengl_hidden_surface_removal_enable"), HTuple("true"));
    ros::init(argc, argv, "halcon_object_detection");

    // Internal used structures to store detected devices
    char **devices = 0;          // List of available devices
    char **callback_types = 0;   // List of available callback types
    int  callback_index = -1;
#ifdef _WIN32
    unsigned int thread_id;     // Thread identifier
#else
    int  error;
#endif

    gThreadRunning = TRUE;
    try
    {
        HFramegrabber acq;

        gAcqCondition = new HCondition("","");
        gAcqMutex = new HMutex("","");

        // Show all available devices
        int num_devices = showAvailableDevices(&devices);
        if(!num_devices)
            return exitProgram(acq,devices,callback_types,num_devices,
                               0,FALSE,callback_index);

        // Select a specific device and call open_framegrabber()
        if(!initDevice(devices,acq,num_devices))
            return exitProgram(acq,devices,callback_types,num_devices,
                               0,FALSE,callback_index);

#ifdef _WIN32
        gThreadHandle = (HANDLE)_beginthreadex(NULL,0,
                                               (unsigned (__stdcall *)(void*))workerThread,
                                               (void*)&acq,0,&thread_id);

        if(gThreadHandle==0)
        {
            cout << "Error creating beginthreadex." << endl;
            gAcqMutex->UnlockMutex();
            return exitProgram(acq,devices,callback_types,num_devices,
                               0,FALSE,callback_index);
        }
#else
        if((error=pthread_create(&gThreadHandle,
                                 NULL,
                                 (void*(*)(void*))workerThread,
                                 (void*)&acq))!=0)
        {
            cout << "Error in pthread_create. Error: " << error << endl;
            return exitProgram(acq,devices,callback_types,num_devices,
                               0,FALSE,callback_index);
        }
#endif

        //    gAcqMutex->LockMutex();
        //    // Show the available callback types and register a specific one
        //    int num_callback_types = showAvailableCallbackTypes(&callback_types,acq);
        //    if(!num_callback_types)
        //    {
        //      gAcqMutex->UnlockMutex();
        //      return exitProgram(acq,devices,callback_types,num_devices,
        //                         num_callback_types,TRUE,callback_index);
        //    }

        //    if(!registerCallbackType(acq,callback_types,num_callback_types,
        //                             callback_index))
        //    {
        //      gAcqMutex->UnlockMutex();
        //      return exitProgram(acq,devices,callback_types,num_devices,
        //                         num_callback_types,TRUE,callback_index);
        //    }

        // Place place your code here, to force the execution of a specific
        // callback type
        cout << "\nIf the registered event is signaled, the previous registered\n"
                " user-specific callback function will be executed.\n\n";

        // e.g. 'ExposureEnd'
        cout << "Start grabbing an image.\n"
                "If e.g. an exposure end was registered, the image grabbing will "
                "force the execution of the user-specific callback function.\n\n";

        try
        {
            acq.GrabImageStart(-1);
        }
        catch(HException &exc)
        {
            cout << exc.ErrorMessage() << endl;
        }

        //    gAcqMutex->UnlockMutex();

        fflush(stdin);

        //        cout << "\n\nType 'y' and 'ENTER' to exit\n\n";
        //        char input;
        //        cin >> input;
        while(true)
            ;
        //            cin >> input;

        //    exitProgram(acq,devices,callback_types,num_devices,
        //                num_callback_types,TRUE,callback_index);
    }

    catch(HOperatorException &exc)
    {
        cout << exc.ErrorMessage() << endl;

        // Check if extended error information available
        if ((exc.ExtendedErrorCode() != 0) || !(exc.ExtendedErrorMessage().IsEmpty()))
        {
            cout << "Extended error code: " << exc.ExtendedErrorCode() << endl;
            cout << "Extended error message: " << exc.ExtendedErrorMessage() << endl;
        }
    }
    return(0);
}

// Display all available devices. The detected device list will be saved.
// The user has the possibility to select a device from the list, and to
// check if the device supports a HALCON Image Acquisition callback
int showAvailableDevices(char ***devices)
{
    HTuple device_inf, device_val;
    InfoFramegrabber(cInterface, "device", &device_inf, &device_val);

    char **tmp_dev;
    *devices = NULL;

    cout << "\nDetect all available devices.\n"
            "Show all detected devices.\n\n";

    int num_devices = (int)device_val.Length();
    if(num_devices > 0 )
        cout << "Available devices: \n";
    else
    {
        cout << "Found no devices.\n";
        return FALSE;
    }

    tmp_dev = new char*[num_devices];
    for(int i=0;i<num_devices; i++)
    {
        tmp_dev[i] = new char[device_val.TupleSelect(i).ToString().Length()+1];
        memcpy(tmp_dev[i],
               device_val.TupleSelect(i).S(),
               device_val.TupleSelect(i).S().Length()+1);
        cout << i+1 << ") " << tmp_dev[i] << "\n";
    }

    *devices = tmp_dev;
    return num_devices;
}

// Open a selected device
bool initDevice(char **devices, HFramegrabber &acq, int num_devices)
{

    cout << "\nEnter the number of the device you want to connect to: ";

    int device_number = readUserSelection(num_devices);

    cout << "\nOpen the specified device by calling open_framegrabber(...).\n";

    //Nikhil:
    //    acq = HFramegrabber(cInterface, 1, 1, 640, 480, 0, 0, "progressive",
    //                        8, "default", -1, "false", cCameraType,
    //                        devices[device_number-1], 0, -1);

    acq = HFramegrabber(cInterface, 1, 1, 0, 0, 0, 0, "progressive",
                        8, "default", -1, "false", cCameraType,
                        devices[device_number-1], 0, -1);
    return TRUE;
}


// Shows the supported HALCON Image Acquisition callbacks
int showAvailableCallbackTypes(char*** callback_types,HFramegrabber &acq)
{
    HTuple value;
    char **tmp_dev;
    *callback_types = NULL;

    cout << "Get the supported callback types using "
            "the parameter 'available_callback_types'.\n"
            "Show the available callback types.\n";

    //  HTuple *Revision;
    //  GetFramegrabberParam(acq, "revision", Revision);

    //  value = acq.GetFramegrabberParam("available_callback_types");

    //  int num_callback_types = (int)value.Length();
    //  if(num_callback_types > 0)
    //    cout << "\nAvailable callback types:\n";
    //  else
    //  {
    //    cout << "\nNo callback types supported by the selected device.\n";
    //    return FALSE;
    //  }

    //  tmp_dev = new char*[num_callback_types];
    //  for(int i=0; i<num_callback_types; i++)
    //  {
    //    tmp_dev[i] = new char[value.TupleSelect(i).ToString().Length()+1];

    //    memcpy(tmp_dev[i],
    //           value.TupleSelect(i).S(),
    //           value.TupleSelect(i).S().Length()+1);

    //    cout << i+1 << ") " << tmp_dev[i] << "\n";
    //  }

    //  *callback_types = tmp_dev;
    //  return num_callback_types;
    return 1;
}

// Register the callback type the user has selected
bool registerCallbackType(HFramegrabber &acq, char **callback_types,
                          int num_callback_types, int &callback_index)
{
    cout << "\nEnter the number of the callback type you like to register: ";

    callback_index = readUserSelection(num_callback_types);

    // e.g. Activate the GigEVision GenApi event's.
    // If the selected callback type isn't based on GenApi,
    // it's recommended to skip the following lines.
    if(!(strcmp(cInterface, "GigEVision")))
    {
        if((strcmp(callback_types[callback_index-1], "transfer_end")) &&
                (strcmp(callback_types[callback_index-1], "device_lost")) &&
                (strcmp(callback_types[callback_index-1], "callback_queue_overflow")))
        {
            acq.SetFramegrabberParam("EventSelector",
                                     callback_types[callback_index-1]);
            // Enable the GeniCam event notification.
            try
            {
                HTuple Values = acq.GetFramegrabberParam("EventNotification_values");
                HString OnVal = Values.TupleSelect(1).ToString();
                acq.SetFramegrabberParam( "EventNotification", OnVal.Text());
            }
            catch(HException &)
            {
                cout << "\nEventNotification_values is not supported by this device,\n"
                        "we try acq.SetFramegrabberParam(\"EventNotification\",\"On\")\n";
                acq.SetFramegrabberParam( "EventNotification", "On");
            }
        }
    }

    // Register the selected callback function, in this example the
    // user-context is the name of the specified callback type.
    // Any other pointers or structures could be used.
    // If you do not need the user-context use NULL
    acq.SetFramegrabberCallback(callback_types[callback_index-1],
                                (void*)MyCallbackFunction,
                                callback_types[callback_index-1]);

    // You can also use get_framegrabber_callback() to query the callback
    // function and the userContext. See the following lines.

    // The following lines are only demonstrating, how the previous
    // user-specifec callback settings could be read.
    void *my_callback_function;  //Pointer to the callback function
    void *my_user_context;       //Previous used userContext
    my_callback_function =
            acq.GetFramegrabberCallback(callback_types[callback_index-1],
                                        &my_user_context);
    return TRUE;
}

// Read the user input. Used by the device and callback type selection.
int readUserSelection(int num_devices)
{
    int number = 0;

    while (!(cin >> number) || number < 1 || number > num_devices)
    {
        cin.clear();
        cin.ignore(1024,'\n');
        cout << "Invalid input.\nSelect a listed entry: ";
    }

    return number;
}

// Cleanup the internal device and callback list. Close the framegrabber.
bool exitProgram(HFramegrabber &acq, char **devices, char **callback_types,
                 int num_devices, int num_callback_types, int init_device,
                 int callback_index)
{
    if(callback_index >= 0)
    {
        gAcqMutex->LockMutex();
        acq.SetFramegrabberCallback(callback_types[callback_index-1],NULL,NULL);
        gAcqMutex->UnlockMutex();
    }

    if(init_device)
    {
        gAcqMutex->LockMutex();
        gThreadRunning = FALSE;
        gAcqCondition->SignalCondition();
        gAcqMutex->UnlockMutex();

#ifdef _WIN32
        if(WaitForSingleObject(gThreadHandle, INFINITE) != WAIT_OBJECT_0)
            cout << "Error in WaitForSingleObject. " << endl;
#else
        if(pthread_join(gThreadHandle, NULL) != 0)
            cout << "Error in pthread_join. " << endl;
#endif

        acq.Clear();
    }

    if(num_devices)
        delete devices;

    if(num_callback_types)
        delete callback_types;

    delete gAcqCondition;
    delete gAcqMutex;

    return TRUE;
}
