// ===========================================================================
/**
 * \file ftkDoxygen.h
 * \brief Documentation main page.
 *
 * This file contains a simple tutorial which presents how the library
 * works, history information.
 *
 * \mainpage
 *
 * \section License
 *
 * These files are part of the Atracsys fusionTrack library.
 *
 * Copyright (C) 2003-2018 by Atracsys LLC. All rights reserved.
 *
 * You can freely use this SDK for your own applications.
 *
 * A license to use binary files of this distribution is granted to
 * owner of infiniTrack, spryTrack and/or fusionTrack devices.
 *
 * It is not allowed to redistribute any source file of this
 * distribution without written permission of Atracsys.
 *
 * \section Release
 *
 * This SDK may be integrated with numerous popular C++ compiler: GCC,
 * Visual Studio, Intel C++ Compiler, etc.
 *
 * Shared libraries of this SDK are available for the following operating
 * systems:
 *  - Windows x64 (starting Windows 8.1, 64 bits version)
 *  - Linux 64 bits
 *
 * \subsection Versioning
 *
 * From release 2.1, the numbering scheme was changed: only the major
 * and minor revision numbers are given, although the full version can
 * be accessed from the SDK. Starting from release 3.0.1 the revision
 * number is again given.
 *
 * \section Description
 *
 * This library enables to acquire high-level tracking data of either
 * an infiniTrack, a fusionTrack or a spryTrack tracking system.
 *
 * Functions are available through a comprehensible C-style shared
 * library.  Given the architecture of the library, functions can
 * easily be wrapped into C++ classes.
 *
 * Functions of this SDK are well documented including comprehensive
 * examples. Alternatively, a first step tutorial can be found in the
 * tutorial section of this page. Samples programs are also a good
 * starting point to understand how to integrate this SDK in your
 * project.
 *
 * \section integrate How to integrate this SDK in your own project
 *
 * Once the driver is successfully installed on your machine, please
 * add the following elements in your C/C++ project:
 *  - Include the path {install folder}/include/ directory in your
 *    project, where {install folder} is your installation directory;
 *  - Add fusionTrackXX.lib, where XX is 32 or 64 depending on your
      compiler;
 *  - Compile and run.
 *
 * \section samples How to compile the samples
 *
 * Perform the steps of previous section: "How to integrate this SDK"
 * and add the following files in your project:
 *  - ftk ... cpp (sample file located in {install folder}/samples
 *    directory)
 *  - helpers.hpp along with corresponding helpers_windows.cpp or
 *    helpers_linux.cpp
 *
 *  Note that for cmake users, all these steps are already prepared in
 *  the CMakeLists.txt file located in {install folder}/samples
 *  directory.
 *
 * \if FTK
 *
 * \section connection Loosing connection
 *
 * If successive calls to ftkGetLastFrame() always return
 * FTK_WAR_NO_FRAME, this is an indication of connection loss. In this
 * case, the physical connection must be checked (unplugged cables for
 * instance), and the device should be re-discovered by calling
 * ftkEnumerateDevices(). The device settings might need to be applied
 * again after the reconnection.
 *
 * \section list List of Samples
 *
 * List of available samples:
 *  - Sample 1: displays device options in the console;
 *  - Sample 2: gets marker pose with MS2 geometry;
 *  - Sample 3: complete acquisition procedure;
 *  - Sample 4: basic acquisition of raw data;
 *  - Sample 5: acquisition and getting the relative position of a marker
 *         with respect to a second one;
 *  - Sample 6: shows how to use the User-LED;
 *  - Sample 7: shows how to extract the actual number of raw data /
 *         fiducials even in case of overflow;
 *  - Sample 8: shows how to get the temperatures;
 *  - Sample 9: shows how to get the data from the accelerometer(s);
 *  - Sample 10: shows how to get the device unix timestamp;
 *  - Sample 11: shows how to activate a wireless marker (if applicable).
 *
 * Samples are located in directory {install folder}/samples
 *
 * \endif
 * \if STK
 *
 * \section list List of Samples
 *
 * List of available samples:
 *  - Sample 1: displays device options in the console;
 *  - Sample 2: gets marker pose with MS2 geometry;
 *  - Sample 3: complete acquisition procedure;
 *  - Sample 4: basic acquisition of raw data;
 *  - Sample 5: acquisition and getting the relative position of a marker
 *         with respect to a second one;
 *  - Sample 6: shows how to use the User-LED;
 *  - Sample 7: shows how to extract the actual number of raw data /
 *         fiducials even in case of overflow;
 *  - Sample 12: shows the last logs of the spryTrack;
 *  - Sample 13: shows how to get the temperatures;
 *  - Sample 14: shows how to track active markers;
 *  - Sample 15: retrieves the shock monitoring data;
 *  - Sample 16: initialize the real time clock (and sync it)
 *  - Sample 17: force an update of a wireless marker firmware (in case the
 * update on the demo did fail.)
 *
 * Samples are located in directory {install folder}/samples
 *
 * \endif

 * \section tutorial Tutorial
 *
 * This section presents a step-by-step implementation of a basic
 * marker pose acquisition, including other main features of the
 * library.
 *
 * To start using the library, you have to initialize it with ftkInit:
 *
 * \code
 * #define ERROR(x) {printf ("%s\n",x); exit (EXIT_FAILURE);}
 *
 * ftkLibrary handle( ftkInit() );
 * if ( handle == 0 )
 * {
 *      ERROR( "Cannot open library" );
 * }
 * \endcode
 *
 * The handle retrieved by the initialization is further used by most
 * function of the SDK.
 *
 * Before exiting your application, you must close the library.
 *
 * \code
 * if ( ftkClose( &handle ) != FTK_OK )
 * {
 *     ERROR ("Cannot close library");
 * }
 * \endcode
 *
 *  After this point the library handle is no longer valid.
 *
 * Note that except the ftkInit, every function of the SDK return
 * either a warning, an error or a success code. Warnings are set as
 * negative values, FTK_OK as 0 and error as positive values.  Error
 * codes are listed in file 'ftkErrors.h'.
 *
 * After library initialization, you can enumerate available devices.
 * Enumeration is performed via a callback mechanism. A user data
 * parameter enables to pass a predefined value. Function accessing
 * devices are using their serial number as identifier.
 *
 * The following example illustrates a typical C++ implementation
 * using the device enumeration callback. User parameter enables to
 * pass a pointer on the DeviceEnumerator instance:
 *
 * \code
 * class DeviceEnumerator
 * {
 * public:
 *     const std::vector<uint64>& enumerateDevices (ftkLibrary lib)
 *     {
 *         _vectorOfDevices.clear();
 *         if ( ftkEnumerateDevices( lib, _callback, this ) != FTK_OK )
 *         {
 *             ERROR( "Cannot enumerate devices" );
 *         }
 *         return _vectorOfDevices;
 *     }
 *
 * protected:
 *     std::vector<uint64> _vectorOfDevices;
 *
 * private:
 *     static void __cdecl _callback (uint64 sn, void* user, ftkDeviceType type)
 *     {
 *         if ( user != 0 )
 *         {
 *             reinterpret_cast< DeviceEnumerator* >( user )->_vectorOfDevices.push_back( sn );
 *         }
 *     }
 * };
 * \endcode
 *
 * A simpler C example:
 *
 * \code
 * void deviceEnumCallback( uint64 sn, void* user, ftkDeviceType type )
 * {
 *     uint64* lastDevice = (uint64*) user;
 *     if ( lastDevice != 0uLL )
 *     {
 *         lastDevice = sn;
 *     }
 * }
 *
 * main ()
 * {
 *     // Initialize library
 *
 *     uint64 sn = 0LL;
 *     if ( ftkEnumerateDevices( lib, deviceEnumCallback, &sn ) != FTK_OK )
 *     {
 *         ERROR( "Cannot enumerate devices" );
 *     }
 *     if ( sn == 0uLL )
 *     {
 *         ERROR ("No device connected");
 *     }
 * \endcode
 *
 * Device, detection or matching parameters can be get/set via
 * options.  There are three types of options: integer, floating
 * points and data.  For numeric options (int and float) it is
 * possible to know their unit, minimum, maximum, default and actual
 * value.
 *
 * Options enumeration process is performed in a similar way as devices:
 *
 * \code
 * void optionEnumerator( uint64 sn, void* user, ftkOptionsInfo* oi )
 * {
 *     printf( "Option (%u)  %s\n", oi->id, oi->name );
 * }
 *
 * main ()
 * {
 *     // Initialize library (see ftkInit() example)
 *     // Get an attached device (see ftkEnumerateDevices() example)
 *
 *     if ( ftkEnumerateOptions( lib, sn, optionEnumerator, NULL ) != FTK_OK )
 *     {
 *         ERROR ("Cannot enumerate options");
 *     }
 * }
 * \endcode
 *
 * Sample 'stereo1_ListOptions.cpp' presents a running version of options
 * enumeration. It is, by the way, the easiest way to list all the
 * available options of the current driver. Note that the options ID
 * \e might change from one revision to the next.
 *
 * Use alternatively the demonstration Software to experience options
 * and their effect on the tracking process.
 *
 * The next step is to start acquiring frames. The ftkFrameQuery
 * structure stores all the required data coming from a pair of
 * images. Depending on its initialization, you can retrieve the raw
 * images, raw data, 3D fiducials data, marker poses, etc. See
 * documentation of ftkFrameQuery to get an exhaustive idea of the
 * data you can collect.
 *
 * The following example illustrates how to initialize the
 * ftkFrameQuery structure to retrieve up to 16 marker poses within
 * the frame and grab the latest available data.
 *
 * \code
 * ftkFrameQuery* fq = ftkCreateFrame();
 * ftkError err( ftkSetFrameOptions( false, false, 0u, 0u, 0u, 16u, fq ) ); // Retrieves up to 16 markers only.
 * if ( err != FTK_OK )
 * {
 *     error( "Cannot initialise frame query" );
 * }
 *
 * // Wait until the last frame is available
 * if ( ftkGetLastFrame( lib, sn, fq, 0 ) != FTK_OK )
 * {
 *     ERROR ("Error acquiring frame");
 * }
 *
 * if ( markersStat == QS_OK )
 * {
 *     for ( uint32 u( 0u ); u < fq->markersCount; ++u )
 *     {
 *          // Do whatever with the marker (fq->markers [u] ...)
 *     }
 * }
 *
 * ftkDeleteFrame( fq );
 * \endcode
 *
 * Please check the following samples:
 *  -# 'stereo2_AcquisitionBasic.cpp' for a running example of how to
 *       retrieve marker translation and registration error;
 *  -# 'stereo3_AcquisitionAdvanced.cpp' shows how to use your own marker
 *       geometry during tracking;
 *  -# 'stereo4_AcquisitionRawData.cpp' shows how to get raw data and
 *       fiducial without any geometry, and how to get the pictures.
 *
 * Do not hesitate to provide us feedback to improve this tutorial.
 *
 * \section history History
 *
 *  - Release 4.1.1 See release notes on Atracsys Support website.
 *  - Release 3.1.1 Internal release.
 *  - Release 3.0.1 Wireless support deactivated, improvements in the
 *    device detection, bug fixes, added signatures for dumping
 *    function, changed ftkClose signature, added extended error
 *    information, extented option interface for further development,
 *    sample 6 is now controlling the user-LED, change event retrieving
 *    mechanism.
 *  - Release 2.3 Reimplemented event management,
 *  - Release 2.2 Added event handling, fixed bug preventing second
 *    marker to be detected.
 *  - Release 2.1 Integration of wireless marker, new versioning scheme.
 *  - Release 2.0.4.15 Preparation for new compression algorithm support.
 *  - Release 2.0.3.150 Implementation of a new segmentation
 *    algorithm, added checks on the received pictures.
 *  - Release 2.0.1.126 Changed FTK_WAR_SEG_OVERFLOW to
 *    FTK_ERR_SEG_OVERFLOW, improvement in the segmentation algorithm,
 *    added a function to get the accelerometer data.
 *  - Release 2.0.1.0 Bug fix in binary option get/set.
 *  - Release 2.0.0.0 Significant changes in the interface.
 *  - Release 1.0.2.0 Centroid computation improved (jitter is now
 *    reduced)

 *  - Release 1.0.2.0 Removing auto geometries and adding a geometry
 *    helper in samples.
 *  - Release 1.0.1.6 Correcting a bug in the temperature compensation
 *    algorithm.
 *  - Release 1.0.1.5 New public options for fusionTrack device.
 *  - Release 1.0.1.5 Correcting a bug in rigid registration algorithm.
 *  - Release 1.0.1.5 Adding support to double for future extension
 *    (now using floatXX).
 *  - Release 1.0.1.0 Updating API for future extensions.
 *  - Release 1.0.1.0 Adding temperature compensation.
 *  - Release 1.0.1.0 Updating options.
 *  - Release 1.0.0.5 Support for the fusionTrack localizer.
 *  - Release 1.0.0.5 Changing ftkGetLastFrame last parameter.
 *  - Release 1.0.0.5 Support for the 5 different MSx geometries.
 *
 */
/*  ===========================================================================
 * */
