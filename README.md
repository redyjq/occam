# occam
The controller code for the Occam Omni Stereo

# Occam API Instructions
* Download the Indigo SDK tarball and uncompress it into a subdirectory (this step has already been completed on the lab computer)
* Replace the `occam/indigosdk-2.0.34/src/omnis5u3mt9v022.cc` file with the `occam/src/omnis5u3mt9v022.cc` file
* Run `cmake .` to construct the necessary makefiles
* Run `make` to build the code
* Run `./bin/occam`. This will try to construct and display a point cloud. It will also save the RGB and XYZ values of all points to the `data/pointcloud.pcd` file. The source code for this is in `src/read_point_cloud.cc`

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

Might make this a cron job later.
