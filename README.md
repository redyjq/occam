# occam
The controller code for the Occam Omni Stereo

# Occam API Instructions

##Usage
###Setup
* Clone the repo.
* Run `cmake .` to construct the necessary makefiles
* Run `make -j8` to build the code

### Recording mode
* Run `./bin/occam`. This will construct combined point clouds from all of the sensors and stitched images and display the point clouds. The point clouds and corresponding stitched images are stored in `data/poincloudn.pcd` and `data/stitchedn.jpg`. The source code for this is in `src/read_point_cloud.cc`.

### Realtime operation
* If you would like to read from the Occam without going through the disk, include the header in `src/read_point_cloud.h`.
* Get a handle on the occam API with `std::pair<OccamDevice*, OccamDeviceList*> occamAPI = initializeOccamAPI();`.
* Retrieve stitched images and point clouds with `getStitchedAndPointCloud(occamAPI.first, cloud, cvImage)`. See `main` in `src/read_pointcloud.cc` for examples of usage.
* When done with the occam, call `disposeOcamAPI(occamAPI);`.

## Notes

###Point Cloud Quality
If the point clouds are of poor quality (too sparse), tuning the `OCCAM_BM_UNIQUENESS_RATIO` in `indigosdk-2.0.15/src/indigo.h` may help. Additionally, if points are cut off beyond a certain distance, tune the `CULL_THRESHOLD` variable in `read_point_cloud.cc`. Alternatively, a more robust stereo algorithm can be easily implemented by including a library, at the expense of performance; see [libelas](http://www.cvlibs.net/software/libelas/). The following email from Xavier has more details:

    The SDK with the camera uses the standard "block matching" algorithm that is directly ported from OpenCV, but there are quite a few others that can be used. Block matching is completely "local", so depends the most on scene texture but is also the fastest. With passive stereo there is a core trade off between accuracy and density vs computation time. A good example of a recent semi-global algorithm is ELAS [1], which can be easily integrated given the rectified images from the SDK. There are also a number of other algorithms included in OpenCV.


###Examples
If you would like to use the Occam in a different way, run `bin/read_images_opencv` (source file in `indigosdk-2.0.15/examples/read_images_opencv.cc`) and press 1 or 2 to browse the different options available. The other examples may also be useful. In order to record multiple types of data, say an OccamImage and an OccamPointCloud concurrently, for example, `indigosdk-2.0.15/examples/read_raw_images.cc` does this (also `src/read_pointcloud.cc`).

###Issues
If you run into issues working with the API, contact Daryl Sew (darylsew@gmail.com) and Xavier Delacour (xavier@occamvisiongroup.com), our Occam support contact.  

# Database API Instructions
Follow instructions [here](https://developers.google.com/drive/v3/web/quickstart/python#prerequisites) for turning on the Drive API. If that page is down, do the following.

##Step 1: Turn on the Drive API

1. Use [this wizard](https://console.developers.google.com/flows/enableapi?apiid=drive) to create or select a project in the Google Developers Console and automatically turn on the API. Click Continue, then Go to credentials.

2. At the top of the page, select the OAuth consent screen tab. Select an Email address, enter a Product name if not already set, and click the Save button.

3. Select the Credentials tab, click the Add credentials button and select OAuth 2.0 client ID.

4. Select the application type Other, enter the name "Drive API Quickstart", and click the Create button.

4. Click OK to dismiss the resulting dialog.

5. Click the file_download (Download JSON) button to the right of the client ID.

6. Move this file to your working directory and rename it client_secret.json.

##Step 2: Install the Google Client Library

Run the following command to install the library using pip:

    pip install --upgrade google-api-python-client

##Step 3: Run database code

    python upload_daemon.py

It might be useful to set this up as a cron job.
