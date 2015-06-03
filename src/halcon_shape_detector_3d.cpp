#include "ros/ros.h"
#include <opencv2/core/core.hpp>

#ifndef __APPLE__
#  include "halconcpp/HalconCpp.h"
#else
#  ifndef HC_LARGE_IMAGES
#    include <HALCONCpp/HalconCpp.h>
#  else
#    include <HALCONCppxl/HalconCpp.h>
#  endif
#endif


#include <pthread.h>


#define MODEL_NUMBER_TOTAL 13


#define cInterface "Video4Linux2"
// "1394IIDC", "ADLINK", "BitFlow",
// "GenICamTL", "LinX", "MILLite",
// "MultiCam", "pylon", "SaperaLT" ...
#define cCameraType "default"
// e.g. Path to camera configuration

using namespace HalconCpp;
using namespace std;


class HALCON_SHAPE_DETECTOR_3D
{
public:

    pthread_t  gThreadHandle;
    HCondition *gAcqCondition;
    HMutex     *gAcqMutex;
    HBOOL      gThreadRunning;
    HBOOL      gCountCallbacks;
    INT  cMaxGrabImage;
    HFramegrabber acq;
    int modelNumber;
    float focus_in_pixel;


    float *camParamArray;
    HTuple *camParams;

    vector< HTuple > shape_model;
    vector< vector< cv::Point3f > > last_position_model;
    vector< vector< cv::Point3f > > last_orientation_model;
    //shape size is the maximum length of the part
    vector< float > shape_size;

    HALCON_SHAPE_DETECTOR_3D(){
        gCountCallbacks=0;
        cMaxGrabImage=10;

        modelNumber = 0;
        camParamArray = (float*)malloc(sizeof(float)*8);

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

    // Open a selected device
    bool initDevice(char **devices, HFramegrabber &acq, int num_devices)
    {

        cout << "\nEnter the number of the device you want to connect to: ";

        int device_number = this->readUserSelection(num_devices);

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

    HTuple fileSelection(char c, float &unit, float &size, int index = -1)
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
             << "11. whole part \n"
             << "12. bluebox_shifted \n"
             << "13. plate"
             <<endl;

        int fileIndex;
        if(index == -1){
            cin >> fileIndex;
        }
        else{
            fileIndex = index;
        }

        while(fileIndex<1 || fileIndex > MODEL_NUMBER_TOTAL){
            cout << "Wrong input! Select again!" << endl;
            cin >> fileIndex;
        };

        HTuple fileName;
        if(fileIndex == 1){
            fileName = HTuple("bearing");
            size = 0.04;
        }
        else if(fileIndex == 2){
            fileName = HTuple("bluebox");
            size = 0.2;
        }
        else if(fileIndex == 3){
            fileName = HTuple("flange screw");
            size = 0.1;
        }
        else if(fileIndex == 4){
            fileName = HTuple("mechanicalpipe");
            size = 0.2;
        }
        else if(fileIndex == 5){
            fileName = HTuple("mechanicaltree");
            size = 0.2;
        }
        else if(fileIndex == 6){
            fileName = HTuple("ring wheel");
            size = 0.2;
        }
        else if(fileIndex == 7){
            fileName = HTuple("screw m5x12 - 8.8");
            size = 0.2;
        }
        else if(fileIndex == 8){
            fileName = HTuple("spacer");
            size = 0.04;
        }
        else if (fileIndex == 9){
            fileName = HTuple("reduction-150-120-fillet-5");
            size = 0.25;
        }
        else if (fileIndex == 10){
            fileName = HTuple("simplifiedBearing");
            size = 0.04;
        }
        else if (fileIndex == 11){
            fileName = HTuple("whole part");
            size = 0.2;
        }
        else if (fileIndex == 12){
            fileName = HTuple("bluebox_shifted");
            size = 0.3;
        }
        else{
            fileName = HTuple("plate");
            size = 0.3;
        }

        unit = 1;
        if (fileIndex == 2 ||fileIndex == 10 || fileIndex == 12 || fileIndex == 13)
            unit = 0.001;


        return fileName;


    }

    int initialization(){
        ROS_INFO("Halcon shape detector initialization.");

        SetSystem(HTuple("opengl_hidden_surface_removal_enable"), HTuple("true"));

        // Internal used structures to store detected devices
        char **devices = 0;          // List of available devices
        char **callback_types = 0;   // List of available callback types
        int  callback_index = -1;

        int  error;


        gThreadRunning = TRUE;
        try
        {


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

            //            if((error=pthread_create(&gThreadHandle,
            //                                     NULL,
            //                                     (void*(*)(void*))workerThread,
            //                                     (void*)&acq))!=0)
            //            {
            //                cout << "Error in pthread_create. Error: " << error << endl;
            //                return exitProgram(acq,devices,callback_types,num_devices,
            //                                   0,FALSE,callback_index);
            //            }



            try
            {
                acq.GrabImageStart(-1);
            }
            catch(HException &exc)
            {
                cout << exc.ErrorMessage() << endl;
            }


            //        fflush(stdin);

            //        while(ros::ok())
            //            ;

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
        return 0;
    }

    void object_detection(){
        HImage image;
        HTuple *status = new HTuple();

        HTuple *object_model = new HTuple();

        image = acq.GrabImageAsync(-1);

        //halcon_install/doc/html/reference/operators/calibrate_cameras.html

        //focus: 1551 pixels
        focus_in_pixel = 1551;
        camParamArray[0] = 0.0198528; //focus
        camParamArray[1] = 0; //Kappa
        camParamArray[2] = 1.28e-005; //Sx
        camParamArray[3] = 1.28e-005; //Sy
        camParamArray[4] = 611,629865; //Cx
        camParamArray[5] = 369,303181; //Cy
        camParamArray[6] = image.Width(); //ImageWidth
        camParamArray[7] = image.Height(); //ImageHeight

        camParams = new HTuple(camParamArray, 8);

        std::cout << "Image size: " << image.Width().ToString() << "*" << image.Height().ToString() << std::endl;



        HTuple *genParam = new HTuple();
        HTuple *genParamValue = new HTuple();

        HTuple start, end, time1;

        cout << "Create a new model?(Y/N)" << endl;
        char c;
        cin >> c;


        while(c == 'Y' || c == 'y'){
            float unit, size;
            HTuple fileName = fileSelection(c, unit, size);


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
                                   -(HTuple(45).TupleRad()),HTuple(45).TupleRad(),
                                   -(HTuple(45).TupleRad()), HTuple(45).TupleRad(),
                                   -HTuple(0).TupleRad(), HTuple(90).TupleRad(),
                                   1.1, 1.2, 10,
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

        //shape size is the maximum length of the part
        cout << "Which model do you want to load?" << endl;
        cout << "6 models at most" << endl;
        c = 'y';
        int modelNumber = 0;
        while(c == 'Y' || c == 'y'){
            float unit, size;
            HTuple fileName = fileSelection(c, unit, size);
            shape_size.push_back(size);
            cv::Point3f initialPosition;
            initialPosition.x = NAN;
            initialPosition.y = NAN;
            initialPosition.z = NAN;
            vector< cv::Point3f > last_position_instance;
            last_position_instance.push_back(initialPosition);
            last_position_model.push_back(last_position_instance);


            cout << "Loading selected model..." << modelNumber<< endl;

            try{
                HTuple selected_shape_model;
                //ReadShapeModel3d(fileName+".sm3", &(shape_model[modelNumber]));
                ReadShapeModel3d(fileName+".sm3", &selected_shape_model);
                shape_model.push_back(selected_shape_model);
            }
            catch(HException &exc)
            {
                cout << exc.ErrorMessage() << endl;
            }
            modelNumber ++;

            cout << "(" << modelNumber << " /6) models have been loaded." << endl;
            cout << "Load another model?(Y/N)" << endl;
            cin >> c;
        }
        gAcqMutex->LockMutex();

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
        hv_MatchingParameterValues[0] = 3;
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

        while(ros::ok())
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
                image = acq.GrabImageAsync(-1);
            }
            catch(HException &exc)
            {
                cout << exc.ErrorMessage() << endl;
            }

            // window.DispObj(image);



            HImage greyImage, greyImageReduced;
            Rgb1ToGray(image, &greyImage);

            HTuple *Pose = new HTuple();
            HTuple *CovPose = new HTuple();
            HTuple *Score = new HTuple();
            //      GrabImageAsync(image, acq, -1);
            //      std::cout << image.Width() << "," << image.Height() << std::endl;
            ROS_INFO("Searching for object...");
            CountSeconds(&start);
            for(int modelIndex = 0; modelIndex < modelNumber; modelIndex++){
                int instance_number = last_position_model[modelIndex].size();
                vector< cv::Point3f > last_position_instance;
                vector< cv::Point3f > last_orientation_instance;
                for(int instance = 0; instance < instance_number; instance++){
                    //using detection result of last frame to reduce searching range:
                    if(isfinite(last_position_model[modelIndex][instance].x)){//last position is valid

                        cv::Point2f center, p1, p2;
                        float size2d;
                        center.x = last_position_model[modelIndex][instance].x/last_position_model[modelIndex][instance].z
                                *focus_in_pixel + camParamArray[4];
                        center.y = last_position_model[modelIndex][instance].y/last_position_model[modelIndex][instance].z
                                *focus_in_pixel + camParamArray[5];
                        size2d = shape_size[modelIndex]/last_position_model[modelIndex][instance].z*focus_in_pixel;
                        p1.x = center.x - size2d;
                        p1.y = center.y - size2d;
                        p2.x = center.x + size2d;
                        p2.y = center.y + size2d;

                        //                    std::cout << "search range: " << p1 << " " << p2 << std::endl;


                        HRegion region(HTuple(p1.y), HTuple(p1.x), HTuple(p2.y), HTuple(p2.x));
                        ReduceDomain(greyImage, region, &greyImageReduced);
                    }
                    else{//last position is invalid
                        ROS_ERROR("Global search!");
                        CopyImage(greyImage, &greyImageReduced);
                    }

                    //            HRegion region(HTuple(0), HTuple(640), HTuple(480), HTuple(1280));
                    //            ReduceDomain(greyImage, region, &greyImageReduced);




                    try{
                        //FindShapeModel3d(greyImage, shape_model[0], 0.65, 0.9, 0, "recompute_score", "true", Pose, CovPose, Score);
                        FindShapeModel3d(greyImageReduced, shape_model[modelIndex], 0.65, 0.9, 0,
                                         hv_MatchingParameters, hv_MatchingParameterValues,
                                         Pose, CovPose, Score);



                    }

                    catch(HException &exc)
                    {
                        cout << exc.ErrorMessage() << endl;
                    }
                    if (modelIndex == 0 && HDevWindowStack::IsOpen())
                        DispObj(greyImage, HDevWindowStack::GetActive());

                    if(Score->TupleLength()>0){
                        //                    std::cout << Score->TupleLength().ToString() << " matches have been found!" << std::endl;
                        //                    for(int i = 0; i < Score->TupleLength(); i++){
                        //                        std::cout << Score->TupleSelect(i).ToString() << "  ";
                        //                    }
                        //                    std::cout << endl;

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
                                //                            ROS_INFO("Displaying object...");
                                if (HDevWindowStack::IsOpen())
                                    SetColor(HDevWindowStack::GetActive(),HTuple(hv_Colors[modelIndex]));
                                DispObj(ho_ModelContours, HDevWindowStack::GetActive());
                            }

                            //                        std::cout << Pose->TupleLength().ToString() << std::endl;
                            //                        for(int ahh = 0; ahh < 7; ahh++){
                            //                            std::cout << Pose->TupleSelect(ahh).ToString() << " ";
                            //                        }
                            //                        std::cout << std::endl;

                            //update last position
                            cv::Point3f posi;
                            posi.x = float(Pose->TupleSelect(hv_J*7));
                            posi.y = float(Pose->TupleSelect(hv_J*7+1));
                            posi.z = float(Pose->TupleSelect(hv_J*7+2));

                            cv::Point3f orientation;
                            orientation.x = float(Pose->TupleSelect(hv_J*7+3));
                            orientation.y = float(Pose->TupleSelect(hv_J*7+4));
                            orientation.z = float(Pose->TupleSelect(hv_J*7+5));

                            cout << "posi " << posi << " ori " << orientation << std::endl;


                            last_position_instance.push_back(posi);
                            last_orientation_instance.push_back(orientation);

                        }
                    }
                    else{//no instance has been found
                        //                    if(last_position_model[modelIndex].size() > 1){
                        //                        last_position_model[modelIndex].erase(last_position_model[modelIndex].begin()+instance);
                        //                    }
                        //                    else{
                        //                    cv::Point3f initialPosition;
                        //                    initialPosition.x = NAN;
                        //                    initialPosition.y = NAN;
                        //                    initialPosition.z = NAN;
                        //                    last_position_model[modelIndex][0] = initialPosition;
                        //                    }
                    }
                }
                last_position_model[modelIndex].clear();
                last_orientation_model[modelIndex].clear();
                if(last_position_instance.size()){
                    for(int instance = 0; instance < last_position_instance.size(); instance++){
                        last_position_model[modelIndex].push_back(last_position_instance[instance]);
                        last_orientation_model[modelIndex].push_back(last_orientation_instance[instance]);
                    }
                }
                else{
                    cv::Point3f initialPosition;
                    initialPosition.x = NAN;
                    initialPosition.y = NAN;
                    initialPosition.z = NAN;
                    last_position_model[modelIndex].push_back(initialPosition);
                    last_orientation_model[modelIndex].push_back(initialPosition);
                }

            }

            float overlappingThreshold = 0.01;
            for(int modelIndex = 0; modelIndex < modelNumber; modelIndex++){
                for(int instanceIndex = 0; instanceIndex < last_position_model[modelIndex].size(); instanceIndex++){
                    for(int candidateIndex = last_position_model[modelIndex].size()-1; candidateIndex >instanceIndex ; candidateIndex --){
                        float abs_x = fabs(last_position_model[modelIndex][instanceIndex].x
                                           - last_position_model[modelIndex][candidateIndex].x);
                        float abs_y = fabs(last_position_model[modelIndex][instanceIndex].y
                                           - last_position_model[modelIndex][candidateIndex].y);
                        float abs_z = fabs(last_position_model[modelIndex][instanceIndex].z
                                           - last_position_model[modelIndex][candidateIndex].z);
                        if(abs_x < overlappingThreshold && abs_y < overlappingThreshold && abs_z  < overlappingThreshold ){
                            last_position_model[modelIndex].erase(last_position_model[modelIndex].begin() + candidateIndex);
                            last_orientation_model[modelIndex].erase(last_orientation_model[modelIndex].begin() + candidateIndex);
                        }
                    }
                }
                if(isfinite(last_position_model[modelIndex][0].x))
                    cout << "Model " << modelIndex << ": " << last_position_model[modelIndex].size() << " instances." << endl;
                else
                    cout << "Model " << modelIndex << ": " << 0 << " instances." << endl;
            }

            CountSeconds(&end);
            time1 = end - start;
            cout << "Find all objects in " << time1.TupleString(".2f").ToString() << " seconds." << endl;

        }
        gAcqMutex->UnlockMutex();
        cout << "End of acquisition thread\n";

    }

    void create_model(){
        HImage image;
        HTuple *status = new HTuple();

        HTuple *object_model = new HTuple();

        image = acq.GrabImageAsync(-1);

        //focus: 1551 pixels
        focus_in_pixel = 1551;
        camParamArray[0] = 0.0198528; //focus
        camParamArray[1] = 0; //Kappa
        camParamArray[2] = 1.28e-005; //Sx
        camParamArray[3] = 1.28e-005; //Sy
        camParamArray[4] = 611,629865; //Cx
        camParamArray[5] = 369,303181; //Cy
        camParamArray[6] = image.Width(); //ImageWidth
        camParamArray[7] = image.Height(); //ImageHeight

        camParams = new HTuple(camParamArray, 8);

        std::cout << "Image size: " << image.Width().ToString() << "*" << image.Height().ToString() << std::endl;

        HTuple *genParam = new HTuple();
        HTuple *genParamValue = new HTuple();

        HTuple start, end, time1;

        cout << "Create a new model?(Y/N)" << endl;
        char c;
        cin >> c;


        while(c == 'Y' || c == 'y'){
            float unit, size;
            HTuple fileName = fileSelection(c, unit, size);


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
                                   -(HTuple(45).TupleRad()),HTuple(45).TupleRad(),
                                   -(HTuple(45).TupleRad()), HTuple(45).TupleRad(),
                                   -HTuple(0).TupleRad(), HTuple(90).TupleRad(),
                                   1.1, 1.2, 10,
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

    }

    void load_model(){


        cout << "Which model do you want to load?" << endl;
        cout << "6 models at most" << endl;
        char c = 'y';

        while(c == 'Y' || c == 'y'){
            float unit, size;
            HTuple fileName = fileSelection(c, unit, size);
            shape_size.push_back(size);
            cv::Point3f initialPosition;
            initialPosition.x = NAN;
            initialPosition.y = NAN;
            initialPosition.z = NAN;
            vector< cv::Point3f > last_position_instance;
            last_position_instance.push_back(initialPosition);
            last_position_model.push_back(last_position_instance);
            last_orientation_model.push_back(last_position_instance);


            cout << "Loading selected model..." << modelNumber<< endl;

            try{
                HTuple selected_shape_model;
                //ReadShapeModel3d(fileName+".sm3", &(shape_model[modelNumber]));
                ReadShapeModel3d(fileName+".sm3", &selected_shape_model);
                shape_model.push_back(selected_shape_model);
            }
            catch(HException &exc)
            {
                cout << exc.ErrorMessage() << endl;
            }
            modelNumber ++;

            cout << "(" << modelNumber << " /6) models have been loaded." << endl;
            cout << "Load another model?(Y/N)" << endl;
            cin >> c;
        }
        gAcqMutex->LockMutex();
    }

    void detect_all_loaded_models(){
        HImage image = acq.GrabImageAsync(-1);
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
        hv_MatchingParameterValues[0] = 3;
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
        hv_Colors[6] = "coral";
        hv_Colors[7] = "goldenrod";
        hv_Colors[8] = "violet";
        hv_Colors[9] = "salmon";
        hv_Colors[10] = "maroon";
        hv_Colors[11] = "medium aquamarine";
        hv_Colors[12] = "wheat";
        hv_Colors[13] = "midnight blue";


        for(int frame = 0; frame < 1; frame++)
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
                image = acq.GrabImageAsync(-1);
            }
            catch(HException &exc)
            {
                cout << exc.ErrorMessage() << endl;
            }

            // window.DispObj(image);



            HImage greyImage, greyImageReduced;
            Rgb1ToGray(image, &greyImage);

            HTuple *Pose = new HTuple();
            HTuple *CovPose = new HTuple();
            HTuple *Score = new HTuple();
            //      GrabImageAsync(image, acq, -1);
            //      std::cout << image.Width() << "," << image.Height() << std::endl;
            ROS_INFO("Searching for object...");

            HTuple start, end, time1;
            CountSeconds(&start);
            for(int modelIndex = 0; modelIndex < modelNumber; modelIndex++){
                int instance_number = last_position_model[modelIndex].size();
                vector< cv::Point3f > last_position_instance;
                vector< cv::Point3f > last_orientation_instance;
                for(int instance = 0; instance < instance_number; instance++){
                    //using detection result of last frame to reduce searching range:
                    if(isfinite(last_position_model[modelIndex][instance].x)){//last position is valid

                        cv::Point2f center, p1, p2;
                        float size2d;
                        center.x = last_position_model[modelIndex][instance].x/last_position_model[modelIndex][instance].z
                                *focus_in_pixel + camParamArray[4];
                        center.y = last_position_model[modelIndex][instance].y/last_position_model[modelIndex][instance].z
                                *focus_in_pixel + camParamArray[5];
                        size2d = shape_size[modelIndex]/last_position_model[modelIndex][instance].z*focus_in_pixel;
                        p1.x = center.x - size2d;
                        p1.y = center.y - size2d;
                        p2.x = center.x + size2d;
                        p2.y = center.y + size2d;

                        //                    std::cout << "search range: " << p1 << " " << p2 << std::endl;


                        HRegion region(HTuple(p1.y), HTuple(p1.x), HTuple(p2.y), HTuple(p2.x));
                        ReduceDomain(greyImage, region, &greyImageReduced);
                    }
                    else{//last position is invalid
                        ROS_ERROR("Global search!");
                        CopyImage(greyImage, &greyImageReduced);
                    }

                    //            HRegion region(HTuple(0), HTuple(640), HTuple(480), HTuple(1280));
                    //            ReduceDomain(greyImage, region, &greyImageReduced);




                    try{
                        //FindShapeModel3d(greyImage, shape_model[0], 0.65, 0.9, 0, "recompute_score", "true", Pose, CovPose, Score);
                        FindShapeModel3d(greyImageReduced, shape_model[modelIndex], 0.65, 0.9, 0,
                                         hv_MatchingParameters, hv_MatchingParameterValues,
                                         Pose, CovPose, Score);



                    }
                    catch(HException &exc)
                    {
                        cout << exc.ErrorMessage() << endl;
                    }
                    if (modelIndex == 0 && HDevWindowStack::IsOpen())
                        DispObj(greyImage, HDevWindowStack::GetActive());
                    if(Score->TupleLength()>0){
                        //                    std::cout << Score->TupleLength().ToString() << " matches have been found!" << std::endl;
                        //                    for(int i = 0; i < Score->TupleLength(); i++){
                        //                        std::cout << Score->TupleSelect(i).ToString() << "  ";
                        //                    }
                        //                    std::cout << endl;

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
                                //                            ROS_INFO("Displaying object...");
                                if (HDevWindowStack::IsOpen())
                                    SetColor(HDevWindowStack::GetActive(),HTuple(hv_Colors[modelIndex]));
                                DispObj(ho_ModelContours, HDevWindowStack::GetActive());
                            }

                            //                        std::cout << Pose->TupleLength().ToString() << std::endl;
                            //                        for(int ahh = 0; ahh < 7; ahh++){
                            //                            std::cout << Pose->TupleSelect(ahh).ToString() << " ";
                            //                        }
                            //                        std::cout << std::endl;

                            //update last position
                            cv::Point3f posi;
                            posi.x = float(Pose->TupleSelect(hv_J*7));
                            posi.y = float(Pose->TupleSelect(hv_J*7+1));
                            posi.z = float(Pose->TupleSelect(hv_J*7+2));

                            cv::Point3f orientation;
                            orientation.x = float(Pose->TupleSelect(hv_J*7+3));
                            orientation.y = float(Pose->TupleSelect(hv_J*7+4));
                            orientation.z = float(Pose->TupleSelect(hv_J*7+5));

                            cout << "posi " << posi << " ori " << orientation << std::endl;

                            last_position_instance.push_back(posi);
                            last_orientation_instance.push_back(orientation);


                        }
                    }
                    else{//no instance has been found
                        //                    if(last_position_model[modelIndex].size() > 1){
                        //                        last_position_model[modelIndex].erase(last_position_model[modelIndex].begin()+instance);
                        //                    }
                        //                    else{
                        //                    cv::Point3f initialPosition;
                        //                    initialPosition.x = NAN;
                        //                    initialPosition.y = NAN;
                        //                    initialPosition.z = NAN;
                        //                    last_position_model[modelIndex][0] = initialPosition;
                        //                    }
                    }
                }
                last_position_model[modelIndex].clear();
                last_orientation_model[modelIndex].clear();
                if(last_position_instance.size()){
                    for(int instance = 0; instance < last_position_instance.size(); instance++){
                        last_position_model[modelIndex].push_back(last_position_instance[instance]);
                        last_orientation_model[modelIndex].push_back(last_orientation_instance[instance]);
                    }
                }
                else{
                    cv::Point3f initialPosition;
                    initialPosition.x = NAN;
                    initialPosition.y = NAN;
                    initialPosition.z = NAN;
                    last_position_model[modelIndex].push_back(initialPosition);
                    last_orientation_model[modelIndex].push_back(initialPosition);
                }

            }

            float overlappingThreshold = 0.01;
            for(int modelIndex = 0; modelIndex < modelNumber; modelIndex++){
                for(int instanceIndex = 0; instanceIndex < last_position_model[modelIndex].size(); instanceIndex++){
                    for(int candidateIndex = last_position_model[modelIndex].size()-1; candidateIndex >instanceIndex ; candidateIndex --){
                        float abs_x = fabs(last_position_model[modelIndex][instanceIndex].x
                                           - last_position_model[modelIndex][candidateIndex].x);
                        float abs_y = fabs(last_position_model[modelIndex][instanceIndex].y
                                           - last_position_model[modelIndex][candidateIndex].y);
                        float abs_z = fabs(last_position_model[modelIndex][instanceIndex].z
                                           - last_position_model[modelIndex][candidateIndex].z);
                        if(abs_x < overlappingThreshold && abs_y < overlappingThreshold && abs_z  < overlappingThreshold ){
                            last_position_model[modelIndex].erase(last_position_model[modelIndex].begin() + candidateIndex);
                            last_orientation_model[modelIndex].erase(last_orientation_model[modelIndex].begin() + candidateIndex);
                        }
                    }
                }
                if(isfinite(last_position_model[modelIndex][0].x))
                    cout << "Model " << modelIndex << ": " << last_position_model[modelIndex].size() << " instances." << endl;
                else
                    cout << "Model " << modelIndex << ": " << 0 << " instances." << endl;
            }

            CountSeconds(&end);
            time1 = end - start;
            cout << "Find all objects in " << time1.TupleString(".2f").ToString() << " seconds." << endl;

        }
        gAcqMutex->UnlockMutex();
    }

    void detect_one_loaded_model(int modelIndex){
        HImage image = acq.GrabImageAsync(-1);
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
        hv_MatchingParameterValues[0] = 3;
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
        hv_Colors[6] = "coral";
        hv_Colors[7] = "goldenrod";
        hv_Colors[8] = "violet";
        hv_Colors[9] = "salmon";
        hv_Colors[10] = "maroon";
        hv_Colors[11] = "medium aquamarine";
        hv_Colors[12] = "wheat";
        hv_Colors[13] = "midnight blue";

        for(int frame = 0; frame < 1; frame++)
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
                image = acq.GrabImageAsync(-1);
            }
            catch(HException &exc)
            {
                cout << exc.ErrorMessage() << endl;
            }

            // window.DispObj(image);



            HImage greyImage, greyImageReduced;
            Rgb1ToGray(image, &greyImage);

            HTuple *Pose = new HTuple();
            HTuple *CovPose = new HTuple();
            HTuple *Score = new HTuple();
            //      GrabImageAsync(image, acq, -1);
            //      std::cout << image.Width() << "," << image.Height() << std::endl;
            ROS_INFO("Searching for object...");

            HTuple start, end, time1;
            CountSeconds(&start);
            if(modelIndex < modelNumber){
                int instance_number = last_position_model[modelIndex].size();
                vector< cv::Point3f > last_position_instance;
                vector< cv::Point3f > last_orientation_instance;
                for(int instance = 0; instance < instance_number; instance++){
                    //using detection result of last frame to reduce searching range:
                    if(isfinite(last_position_model[modelIndex][instance].x)){//last position is valid

                        cv::Point2f center, p1, p2;
                        float size2d;
                        center.x = last_position_model[modelIndex][instance].x/last_position_model[modelIndex][instance].z
                                *focus_in_pixel + camParamArray[4];
                        center.y = last_position_model[modelIndex][instance].y/last_position_model[modelIndex][instance].z
                                *focus_in_pixel + camParamArray[5];
                        size2d = shape_size[modelIndex]/last_position_model[modelIndex][instance].z*focus_in_pixel;
                        p1.x = center.x - size2d;
                        p1.y = center.y - size2d;
                        p2.x = center.x + size2d;
                        p2.y = center.y + size2d;

                        //                    std::cout << "search range: " << p1 << " " << p2 << std::endl;


                        HRegion region(HTuple(p1.y), HTuple(p1.x), HTuple(p2.y), HTuple(p2.x));
                        ReduceDomain(greyImage, region, &greyImageReduced);
                    }
                    else{//last position is invalid
                        ROS_ERROR("Global search!");
                        CopyImage(greyImage, &greyImageReduced);
                    }

                    //            HRegion region(HTuple(0), HTuple(640), HTuple(480), HTuple(1280));
                    //            ReduceDomain(greyImage, region, &greyImageReduced);




                    try{
                        //FindShapeModel3d(greyImage, shape_model[0], 0.65, 0.9, 0, "recompute_score", "true", Pose, CovPose, Score);
                        FindShapeModel3d(greyImageReduced, shape_model[modelIndex], 0.65, 0.9, 0,
                                         hv_MatchingParameters, hv_MatchingParameterValues,
                                         Pose, CovPose, Score);



                    }
                    catch(HException &exc)
                    {
                        cout << exc.ErrorMessage() << endl;
                    }

                    DispObj(greyImage, HDevWindowStack::GetActive());

                    if(Score->TupleLength()>0){
                        //                    std::cout << Score->TupleLength().ToString() << " matches have been found!" << std::endl;
                        //                    for(int i = 0; i < Score->TupleLength(); i++){
                        //                        std::cout << Score->TupleSelect(i).ToString() << "  ";
                        //                    }
                        //                    std::cout << endl;

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
                                //                            ROS_INFO("Displaying object...");
                                if (HDevWindowStack::IsOpen())
                                    SetColor(HDevWindowStack::GetActive(),HTuple(hv_Colors[modelIndex]));
                                DispObj(ho_ModelContours, HDevWindowStack::GetActive());
                            }

                            //                        std::cout << Pose->TupleLength().ToString() << std::endl;
                            //                        for(int ahh = 0; ahh < 7; ahh++){
                            //                            std::cout << Pose->TupleSelect(ahh).ToString() << " ";
                            //                        }
                            //                        std::cout << std::endl;

                            //update last position
                            cv::Point3f posi;
                            posi.x = float(Pose->TupleSelect(hv_J*7));
                            posi.y = float(Pose->TupleSelect(hv_J*7+1));
                            posi.z = float(Pose->TupleSelect(hv_J*7+2));

                            cv::Point3f orientation;
                            orientation.x = float(Pose->TupleSelect(hv_J*7+3));
                            orientation.y = float(Pose->TupleSelect(hv_J*7+4));
                            orientation.z = float(Pose->TupleSelect(hv_J*7+5));

                            cout << "posi " << posi << " ori " << orientation << std::endl;

                            last_position_instance.push_back(posi);
                            last_orientation_instance.push_back(orientation);


                        }
                    }
                    else{//no instance has been found
                        //                    if(last_position_model[modelIndex].size() > 1){
                        //                        last_position_model[modelIndex].erase(last_position_model[modelIndex].begin()+instance);
                        //                    }
                        //                    else{
                        //                    cv::Point3f initialPosition;
                        //                    initialPosition.x = NAN;
                        //                    initialPosition.y = NAN;
                        //                    initialPosition.z = NAN;
                        //                    last_position_model[modelIndex][0] = initialPosition;
                        //                    }
                    }
                }
                last_position_model[modelIndex].clear();
                last_orientation_model[modelIndex].clear();
                if(last_position_instance.size()){
                    for(int instance = 0; instance < last_position_instance.size(); instance++){
                        last_position_model[modelIndex].push_back(last_position_instance[instance]);
                        last_orientation_model[modelIndex].push_back(last_orientation_instance[instance]);
                    }
                }
                else{
                    cv::Point3f initialPosition;
                    initialPosition.x = NAN;
                    initialPosition.y = NAN;
                    initialPosition.z = NAN;
                    last_position_model[modelIndex].push_back(initialPosition);
                    last_orientation_model[modelIndex].push_back(initialPosition);
                }

            }

            else{
                ROS_ERROR("Invalid model index!");
            }
            float overlappingThreshold = 0.01;
            if(modelIndex < modelNumber){
                for(int instanceIndex = 0; instanceIndex < last_position_model[modelIndex].size(); instanceIndex++){
                    for(int candidateIndex = last_position_model[modelIndex].size()-1; candidateIndex >instanceIndex ; candidateIndex --){
                        float abs_x = fabs(last_position_model[modelIndex][instanceIndex].x
                                           - last_position_model[modelIndex][candidateIndex].x);
                        float abs_y = fabs(last_position_model[modelIndex][instanceIndex].y
                                           - last_position_model[modelIndex][candidateIndex].y);
                        float abs_z = fabs(last_position_model[modelIndex][instanceIndex].z
                                           - last_position_model[modelIndex][candidateIndex].z);
                        if(abs_x < overlappingThreshold && abs_y < overlappingThreshold && abs_z  < overlappingThreshold ){
                            last_position_model[modelIndex].erase(last_position_model[modelIndex].begin() + candidateIndex);
                            last_orientation_model[modelIndex].erase(last_orientation_model[modelIndex].begin() + candidateIndex);
                        }
                    }
                }
                if(isfinite(last_position_model[modelIndex][0].x))
                    cout << "Model " << modelIndex << ": " << last_position_model[modelIndex].size() << " instances." << endl;
                else
                    cout << "Model " << modelIndex << ": " << 0 << " instances." << endl;
            }

            CountSeconds(&end);
            time1 = end - start;
            cout << "Find all objects in " << time1.TupleString(".2f").ToString() << " seconds." << endl;

        }
        gAcqMutex->UnlockMutex();
    }

    void detect_one_loaded_model_without_visualization(int modelIndex){
        HImage image = acq.GrabImageAsync(-1);

        HTuple  hv_MatchingParameters, hv_MatchingParameterValues;
        hv_MatchingParameters.Clear();
        hv_MatchingParameters[0] = "num_matches";
        //hv_MatchingParameters[1] = "pose_refinement";
        hv_MatchingParameters[1] = "max_overlap";
        hv_MatchingParameterValues.Clear();
        hv_MatchingParameterValues[0] = 3;
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
        hv_Colors[6] = "coral";
        hv_Colors[7] = "goldenrod";
        hv_Colors[8] = "violet";
        hv_Colors[9] = "salmon";
        hv_Colors[10] = "maroon";
        hv_Colors[11] = "medium aquamarine";
        hv_Colors[12] = "wheat";
        hv_Colors[13] = "midnight blue";

        for(int frame = 0; frame < 1; frame++)
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
                image = acq.GrabImageAsync(-1);
            }
            catch(HException &exc)
            {
                cout << exc.ErrorMessage() << endl;
            }

            // window.DispObj(image);



            HImage greyImage, greyImageReduced;
            Rgb1ToGray(image, &greyImage);

            HTuple *Pose = new HTuple();
            HTuple *CovPose = new HTuple();
            HTuple *Score = new HTuple();
            //      GrabImageAsync(image, acq, -1);
            //      std::cout << image.Width() << "," << image.Height() << std::endl;
            ROS_INFO("Searching for object...");

            HTuple start, end, time1;
            CountSeconds(&start);
            if(modelIndex < modelNumber){
                int instance_number = last_position_model[modelIndex].size();
                vector< cv::Point3f > last_position_instance;
                vector< cv::Point3f > last_orientation_instance;
                for(int instance = 0; instance < instance_number; instance++){
                    //using detection result of last frame to reduce searching range:
                    if(isfinite(last_position_model[modelIndex][instance].x)){//last position is valid

                        cv::Point2f center, p1, p2;
                        float size2d;
                        center.x = last_position_model[modelIndex][instance].x/last_position_model[modelIndex][instance].z
                                *focus_in_pixel + camParamArray[4];
                        center.y = last_position_model[modelIndex][instance].y/last_position_model[modelIndex][instance].z
                                *focus_in_pixel + camParamArray[5];
                        size2d = shape_size[modelIndex]/last_position_model[modelIndex][instance].z*focus_in_pixel;
                        p1.x = center.x - size2d;
                        p1.y = center.y - size2d;
                        p2.x = center.x + size2d;
                        p2.y = center.y + size2d;

                        //                    std::cout << "search range: " << p1 << " " << p2 << std::endl;


                        HRegion region(HTuple(p1.y), HTuple(p1.x), HTuple(p2.y), HTuple(p2.x));
                        ReduceDomain(greyImage, region, &greyImageReduced);
                    }
                    else{//last position is invalid
                        ROS_ERROR("Global search!");
                        CopyImage(greyImage, &greyImageReduced);
                    }

                    //            HRegion region(HTuple(0), HTuple(640), HTuple(480), HTuple(1280));
                    //            ReduceDomain(greyImage, region, &greyImageReduced);




                    try{
                        //FindShapeModel3d(greyImage, shape_model[0], 0.65, 0.9, 0, "recompute_score", "true", Pose, CovPose, Score);
                        FindShapeModel3d(greyImageReduced, shape_model[modelIndex], 0.65, 0.9, 0,
                                         hv_MatchingParameters, hv_MatchingParameterValues,
                                         Pose, CovPose, Score);



                    }
                    catch(HException &exc)
                    {
                        cout << exc.ErrorMessage() << endl;
                    }


                    if(Score->TupleLength()>0){
                        //                    std::cout << Score->TupleLength().ToString() << " matches have been found!" << std::endl;
                        //                    for(int i = 0; i < Score->TupleLength(); i++){
                        //                        std::cout << Score->TupleSelect(i).ToString() << "  ";
                        //                    }
                        //                    std::cout << endl;

                        //Visualize the found matches in the image by
                        //projecting the 3D shape model with the pose of the match
                        HTuple end_val58 = (Score->TupleLength())-1;
                        HTuple step_val58 = 1;

                        for (HTuple hv_J=0; hv_J.Continue(end_val58, step_val58); hv_J += step_val58)
                        {

                            //update last position
                            cv::Point3f posi;
                            posi.x = float(Pose->TupleSelect(hv_J*7));
                            posi.y = float(Pose->TupleSelect(hv_J*7+1));
                            posi.z = float(Pose->TupleSelect(hv_J*7+2));

                            cv::Point3f orientation;
                            orientation.x = float(Pose->TupleSelect(hv_J*7+3));
                            orientation.y = float(Pose->TupleSelect(hv_J*7+4));
                            orientation.z = float(Pose->TupleSelect(hv_J*7+5));

                            cout << "posi " << posi << " ori " << orientation << std::endl;

                            last_position_instance.push_back(posi);
                            last_orientation_instance.push_back(orientation);


                        }
                    }
                    else{//no instance has been found
                        //                    if(last_position_model[modelIndex].size() > 1){
                        //                        last_position_model[modelIndex].erase(last_position_model[modelIndex].begin()+instance);
                        //                    }
                        //                    else{
                        //                    cv::Point3f initialPosition;
                        //                    initialPosition.x = NAN;
                        //                    initialPosition.y = NAN;
                        //                    initialPosition.z = NAN;
                        //                    last_position_model[modelIndex][0] = initialPosition;
                        //                    }
                    }
                }
                last_position_model[modelIndex].clear();
                last_orientation_model[modelIndex].clear();
                if(last_position_instance.size()){
                    for(int instance = 0; instance < last_position_instance.size(); instance++){
                        last_position_model[modelIndex].push_back(last_position_instance[instance]);
                        last_orientation_model[modelIndex].push_back(last_orientation_instance[instance]);
                    }
                }
                else{
                    cv::Point3f initialPosition;
                    initialPosition.x = NAN;
                    initialPosition.y = NAN;
                    initialPosition.z = NAN;
                    last_position_model[modelIndex].push_back(initialPosition);
                    last_orientation_model[modelIndex].push_back(initialPosition);
                }

            }

            else{
                ROS_ERROR("Invalid model index!");
            }
            float overlappingThreshold = 0.01;
            if(modelIndex < modelNumber){
                for(int instanceIndex = 0; instanceIndex < last_position_model[modelIndex].size(); instanceIndex++){
                    for(int candidateIndex = last_position_model[modelIndex].size()-1; candidateIndex >instanceIndex ; candidateIndex --){
                        float abs_x = fabs(last_position_model[modelIndex][instanceIndex].x
                                           - last_position_model[modelIndex][candidateIndex].x);
                        float abs_y = fabs(last_position_model[modelIndex][instanceIndex].y
                                           - last_position_model[modelIndex][candidateIndex].y);
                        float abs_z = fabs(last_position_model[modelIndex][instanceIndex].z
                                           - last_position_model[modelIndex][candidateIndex].z);
                        if(abs_x < overlappingThreshold && abs_y < overlappingThreshold && abs_z  < overlappingThreshold ){
                            last_position_model[modelIndex].erase(last_position_model[modelIndex].begin() + candidateIndex);
                            last_orientation_model[modelIndex].erase(last_orientation_model[modelIndex].begin() + candidateIndex);
                        }
                    }
                }
                if(isfinite(last_position_model[modelIndex][0].x))
                    cout << "Model " << modelIndex << ": " << last_position_model[modelIndex].size() << " instances." << endl;
                else
                    cout << "Model " << modelIndex << ": " << 0 << " instances." << endl;
            }

            CountSeconds(&end);
            time1 = end - start;
            cout << "Find all objects in " << time1.TupleString(".2f").ToString() << " seconds." << endl;

        }
        gAcqMutex->UnlockMutex();
    }

    void create_all_models(){
        HImage image;
        HTuple *status = new HTuple();

        HTuple *object_model = new HTuple();

        image = acq.GrabImageAsync(-1);

        //focus: 1551 pixels
        focus_in_pixel = 1551;
        camParamArray[0] = 0.0198528; //focus
        camParamArray[1] = 0; //Kappa
        camParamArray[2] = 1.28e-005; //Sx
        camParamArray[3] = 1.28e-005; //Sy
        camParamArray[4] = 611,629865; //Cx
        camParamArray[5] = 369,303181; //Cy
        camParamArray[6] = image.Width(); //ImageWidth
        camParamArray[7] = image.Height(); //ImageHeight

        camParams = new HTuple(camParamArray, 8);

        std::cout << "Image size: " << image.Width().ToString() << "*" << image.Height().ToString() << std::endl;

        HTuple *genParam = new HTuple();
        HTuple *genParamValue = new HTuple();

        HTuple start, end, time1;

        cout << "Create all models?(Y/N)" << endl;
        char c;
        cin >> c;


        if(c == 'Y' || c == 'y'){
            for(int i = 0; i < MODEL_NUMBER_TOTAL; i++){
                float unit, size;
                HTuple fileName = fileSelection(c, unit, size, i+1);


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
                                       -(HTuple(45).TupleRad()),HTuple(45).TupleRad(),
                                       -(HTuple(45).TupleRad()), HTuple(45).TupleRad(),
                                       -HTuple(0).TupleRad(), HTuple(90).TupleRad(),
                                       1.1, 1.2, 10,
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
        }

    }

    void load_all_models(){

        cout << "Load all models..." << endl;
        char c = 'y';

        if(c == 'Y' || c == 'y'){
            for(int i = 0; i < MODEL_NUMBER_TOTAL; i++){
                float unit, size;
                HTuple fileName = fileSelection(c, unit, size, i+1);
                shape_size.push_back(size);
                cv::Point3f initialPosition;
                initialPosition.x = NAN;
                initialPosition.y = NAN;
                initialPosition.z = NAN;
                vector< cv::Point3f > last_position_instance;
                last_position_instance.push_back(initialPosition);
                last_position_model.push_back(last_position_instance);
                last_orientation_model.push_back(last_position_instance);


                cout << "Loading selected model..." << modelNumber+1<< endl;

                try{
                    HTuple selected_shape_model;
                    //ReadShapeModel3d(fileName+".sm3", &(shape_model[modelNumber]));
                    ReadShapeModel3d(fileName+".sm3", &selected_shape_model);
                    shape_model.push_back(selected_shape_model);
                }
                catch(HException &exc)
                {
                    cout << exc.ErrorMessage() << endl;
                }
                modelNumber ++;
            }
        }
        gAcqMutex->LockMutex();
    }

    void reset(){

        shape_model.clear();
        last_position_model.clear();
        shape_size.clear();

        modelNumber = 0;

        initialization();

    }
};
