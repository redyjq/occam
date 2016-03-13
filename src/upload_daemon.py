import requests
import os
import apiclient
from driveapi import get_credentials
from apiclient import discovery
from apiclient.http import MediaFileUpload
import httplib2
import time

# See https://developers.google.com/drive/v3/web/quickstart/python
# for setup guide. Need to have client_secret.json file.

def retry_upload(filename: str, request):
    response = None
    consecutive_error_count = 0
    while response is None:
        try: 
            status, response = request.next_chunk()
        except apiclient.errors.HttpError as e:
            if e.resp.status in [404]:
                # Start the upload all over again
                return False
            if e.resp.status in [500, 502, 503, 504]:
                # Start the upload all over again
                consecutive_error_count += 1
                time.sleep(2**consecutive_error_count)
                continue
            else:
                # Do not retry. Log the error and fail
                raise Exception("Failed to upload %s; %s" % (filename, e.resp))
        consecutive_error_count = 0
        if status:
            print("Uploaded %d%% of %s." % (int(status.progress() * 100), filename))
    print("Completed upload of %s." % filename)
    return True

def upload(folder_id: str, local_folder_path: str, filename: str, service):
    """Uploads a file to folder_id in the authenticated service object.

    In the event of network issues, retries uploading indefinitely. Minimizes
    losses in the event of a disruption in network connectivity. 
    """
    filepath = "%s/%s" % (local_folder_path, filename)
    media = MediaFileUpload(filepath, mimetype='application/x-pcl', resumable=True)
    file_metadata = {
        'name': filename,
        'parents': [ folder_id ]
    }
    request = service.files().create(media_body=media, body=file_metadata)

    # We might also want to return false if this takes too long
    success = False
    while not success:
        try:
            success = retry_upload(filename, request)
        except e:
            print("Failed to upload %s; %s" % e)
            return False
    return True
    
def send_to_drive(local_folder_path: str, server_folder: str):
    """Sends all files in the local folder to the server.

    After operation is complete, deletes files to save disk space.
    """
    credentials = get_credentials()
    http = credentials.authorize(httplib2.Http())
    service = discovery.build('drive', 'v3', http=http)

    results = service.files().list().execute()
    items = results.get('files', [])
    if not items:
        print("No Drive folder named %s found!" % server_folder)
        return
    folders = [f for f in items if 'folder' in f['mimeType']]
    folders = [f for f in items if f['name'] == 'occamdb']
    if len(folders) < 1:
        print("No Drive folder named %s found!" % server_folder)
        return
    folder_id = folders[0]

    files = os.listdir(local_folder_path)
    if len(files) < 1:
        print("No files found in %s!" % local_folder_path)

    for filename in files:
        success = upload(folder_id, local_folder_path, filename, service)
        if success:
            os.remove("%s/%s" % (local_folder_path, filename))

def main():
    local_folder = 'occamdb'
    server_folder = 'occamdb'
    send_to_drive(local_folder, server_folder)
    # TODO: call send_to_drive repeatedly based on remaining disk space

if __name__ == '__main__':
    main()

