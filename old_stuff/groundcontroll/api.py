from flask import Flask, send_file, request, jsonify, render_template
import requests as py_requests
from flask_restful import Resource, Api, reqparse, abort, inputs
from werkzeug.security import safe_str_cmp
from werkzeug.utils import secure_filename
from werkzeug.exceptions import HTTPException, default_exceptions
from werkzeug.datastructures import FileStorage
from functools import wraps
from mimetypes import guess_extension
import os
import numpy as np
import uuid
import predict as prd

def debug(message):
    print('DEBUG: {}'.format(message))

def file_ext(str):
    f,e = os.path.splitext(str)
    return e.lower()

# Checks if filename is allowed
def allowed_ext(ext):
    return ext.lower() in [".jpg", ".png", ".jpe"]

def get_file(args):
    unique_filename = str(uuid.uuid4())
    file_with_path_no_ext = os.path.join("/Images", unique_filename)
    ext = None

    print (args)
    # uploaded as multipart data
    if args['file']:
        file = args['file']
        ext = file_ext(file.filename)
        if file.filename and allowed_ext(ext):
            file.save(file_with_path_no_ext+ext)
        else:
            abort (500, msg='Bad file type {}'.format(file.filename))

    # passed as a payload url
    elif args['url']:
        url = args['url']
        debug ('Got url:{}'.format(url))
        ext = file_ext(url)
        r = py_requests.get(url, allow_redirects=True)
        
        cd = r.headers.get('content-disposition')
        ct = r.headers.get('content-type')
        if cd:
            ext = file_ext(cd)
            debug ('extension {} derived from {}'.format(ext,cd))
        elif ct:
            ext = guess_extension(ct.partition(';')[0].strip())
            if ext == '.jpe': 
                ext = '.jpg'
            debug ('extension {} derived from {}'.format(ext,ct))
            if not allowed_ext(ext):
                abort(400, msg='filetype {} not allowed'.format(ext))        
        else:
            ext = '.jpg'
        open(file_with_path_no_ext+ext, 'wb').write(r.content)
    else:
        abort(400, msg='could not determine file type')

    debug ('get_file returned: {}{}'.format(file_with_path_no_ext,ext))

    return file_with_path_no_ext, ext

def parse_args():

    parser = reqparse.RequestParser()
    parser.add_argument('type', location='args',  default=None)
    parser.add_argument('delete', location='args',
                        type=inputs.boolean, default=False)
    parser.add_argument('download', location='args',
                        type=inputs.boolean, default=False)
    parser.add_argument('url', default=False)
    parser.add_argument('file', type=FileStorage, location='files')

    return parser.parse_args()

class Predict(Resource):
    
    def post(self):

        args = parse_args()
      
        if args['type'] in [None, 'object']:
            p = prd.Predict()
        else:
            abort(400, msg='Invalid Model:{}'.format(args['type']))
            
        fip,ext = get_file(args)
        detections = p.predict(fip,ext, args)

        return detections


app = Flask(__name__)

#Overrides the default http exception handler to return JSON.
def get_http_exception_handler(app):

    handle_http_exception = app.handle_http_exception

    @wraps(handle_http_exception)
    def ret_val(exception):

        exc = handle_http_exception(exception)
        return jsonify({'code': exc.code, 'msg': exc.description}), exc.code

    return ret_val

app.handle_http_exception = get_http_exception_handler(app)

api = Api(app, prefix='/api/model')
app.config['UPLOAD_FOLDER'] = "/Images"
app.config['MAX_CONTENT_LENGTH'] = 5 * 1024 * 1024
app.config['PROPAGATE_EXCEPTIONS'] = True
app.debug = False

api.add_resource(Predict, '/predict')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)