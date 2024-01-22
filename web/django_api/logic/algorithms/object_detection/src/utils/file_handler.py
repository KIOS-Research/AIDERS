import csv
import json
from pathlib import Path
import os
from datetime import datetime


def make_json(csvFilePath, jsonFilePath):
    # create a dictionary
    data = []

    # Open a csv reader called DictReader
    with open(csvFilePath, encoding='utf-8') as csvf:
        csvReader = csv.DictReader(csvf)

        # Convert each row into a dictionary
        # and add it to data
        for rows in csvReader:
            # Assuming a column named 'No' to
            # be the primary key
            data.append(rows)
            # key = rows[list(rows.keys())[0]]
            # data[key] = rows

    # Open a json writer, and use the json.dumps()
    # function to dump data
    with open(jsonFilePath, 'w', encoding='utf-8') as jsonf:
        jsonf.write(json.dumps(data, indent=4))


class File_handler:

    def __init__(self,folder_name,filename,extension, api_url):
        self.videoinfo_path = None
        self.vehicles_path = None
        self.tracks_path = None
        self.api_url = api_url
        self.folder_name = folder_name
        self.filename = filename
        self.extension = extension
        now = datetime.now()
        self.timetmp = now.strftime("%Y-%m-%dT%H:%M")
        self.logfile = os.path.join(folder_name,f'verbose_log_{self.timetmp}.log')
        self.videoinfo = None
        self.vehicles  = None
        self.tracks    = None
        self.check_folder()
        self.create_files()
        self.index = 0

        self.dyna_line = None
        self.line_count = None
        self.dyna_queue = None
        self.queue_count = None




    # creates the 3 needed files for the monitoring analysis
    def create_files(self):

        self.videoinfo_path = f'{self.folder_name}/{self.filename}_videoinfo_{self.timetmp}.{self.extension}'
        videoinfo = open(self.videoinfo_path, 'w')
        videoinfo.write('Name,framerate,width,height,duration,altitude,ppmX,ppmY,realW,realH,lat,long,timestamp\n')
        videoinfo.close()
        self.videoinfo = open(self.videoinfo_path, 'a')

        self.vehicles_path = f'{self.folder_name}/{self.filename}_vehicles_{self.timetmp}.{self.extension}'
        vehicles = open(self.vehicles_path, 'w')
        vehicles.write('Veh_id,class_id,initX,initY,initW,initH,initFrame\n')
        vehicles.close()
        self.vehicles = open(self.vehicles_path, 'a')

        self.tracks_path = f'{self.folder_name}/{self.filename}_tracks_{self.timetmp}.{self.extension}'
        tracks = open(self.tracks_path, 'w')
        tracks.write('Veh_id,frame,tlX,tlY,width,height,direction,V,Vx,Vy,a,aX,aY\n')
        tracks.close()
        self.tracks = open(self.tracks_path, 'a')

    # creates the export folder if not exists
    def check_folder(self):
        Path(self.folder_name).mkdir(parents=True, exist_ok=True)

    def export_track(self,track,resize_to):
        tl = track.tlbr[:2] / resize_to
        br = track.tlbr[2:] / resize_to
        w, h = br - tl

        #'Veh_id,frane,tlX,tlY,width,height,direction,velocity
        V, Vx, Vy, f  = track.traffic.velocities[-1]
        A, Ax, Ay     = track.traffic.accelerations[-1]
        self.tracks.write(f'{track.trk_id},{track.end_frame},{tl[0]:.6f},{tl[1]:.6f},'
                      f'{w:.6f},{h:.6f},{track.direction},{V:.3f},{Vx:.3f},{Vy:.3f},'
                          f'{A:.3f},{Ax:.3f},{Ay:.3f}\n')
        self.index +=1
    def export_vehicle(self,track,resize_to):
        tl = track.bboxes[0][:2] / resize_to
        br = track.bboxes[0][2:] / resize_to
        w, h = br - tl

        #Veh_id,class,initX,initY,initW,initH,initFrame

        self.vehicles.write(f'{track.trk_id},{track.lbl_str},{tl[0]:.6f},{tl[1]:.6f},'
                      f'{w:.6f},{h:.6f},{track.frame_ids[0]}\n')

    def export_videoinfo(self, videoname,stream_fps, size, duration,altitude,latitude=0,longitude=0, ppkm=0):
        """
        params:
        Name : filename
        framerate : stream/file framerate
        duration : total duration of the steram/video
        width : stream width resolution
        height : stream height resolution
        ppmX : pixel per meter for the X axis (width)
        ppmY : pixel per meter for the Y axis (height)
        real_width(m) : real width distance in meters
        real_height(m) : real height distance in meters
        """
        ppmx,ppmy = ppkm[0]/1000,ppkm[1]/1000

        real_width = size[0] / ppmx
        real_height = size[1] / ppmy
        vid_filename = os.path.basename(videoname)
        self.videoinfo.write(f'{vid_filename},{stream_fps:.2f},{size[0]},'
                             f'{size[1]},{duration},{altitude:.2f},{ppmx:.2f},'
                             f'{ppmy:.2f},{real_width:.2f},{real_height:.2f},'
                             f'{latitude:.6f},{longitude:.6f},{self.timetmp}\n')

#TODO normalization of xy
    def add_line(self, id, xy1, xy2, frame_id):
        if self.dyna_line == None and self.line_count == None:
            linespth = f'{self.folder_name}/{self.filename}_lines_{self.timetmp}.{self.extension}'
            dyn_linespth = f'{self.folder_name}/{self.filename}_lines_count_{self.timetmp}.{self.extension}'

            lines = open(linespth, 'w')
            lines.write('line_id, bbx1, bby1, bbx2, bby2, frame\n')
            lines.close()
            self.dyna_line = open(linespth, 'a')

            dyn_lines = open(dyn_linespth, 'w')
            dyn_lines.write('line_id, veh_id, type, frame, total\n')
            dyn_lines.close()
            self.line_count = open(dyn_linespth, 'a')

        self.dyna_line.write(f'{id},{xy1[0]},{xy1[1]},{xy2[0]},{xy2[1]},{frame_id}\n')

    def add_line_count(self, line_id, veh_id,type,frame,total):
        # if self.lines is not None and self.dyna_lines is not None:
        self.line_count.write(f'{line_id},{veh_id},{type},{frame},{total}\n')

    # QUEUE FILE IMPLEMENTATION
    # TODO normalization of xy
    def add_queue(self, id, xy1, xy2, xy3, xy4, frame_id):
        if self.dyna_queue == None and self.queue_count == None:
            queuepth = f'{self.folder_name}/{self.filename}_queues_{self.timetmp}.{self.extension}'
            dyn_queuespth = f'{self.folder_name}/{self.filename}_queues_count_{self.timetmp}.{self.extension}'

            queues = open(queuepth, 'w')
            queues.write('queue_id, ptx1, pty1, ptx2, pty2, ptx3, pty3, ptx4, pty4, frame\n')
            queues.close()
            self.dyna_queue = open(queuepth, 'a')

            dyn_queues = open(dyn_queuespth, 'w')
            dyn_queues.write('queue_id, veh_id, type, frame, total, status\n')
            dyn_queues.close()
            self.queue_count = open(dyn_queuespth, 'a')

        self.dyna_queue.write(f'{id},{xy1[0]},{xy1[1]},{xy2[0]},{xy2[1]},'
                              f'{xy3[0]},{xy3[1]},{xy4[0]},{xy4[1]},{frame_id}\n')

    def add_queue_count(self, queue_id, veh_id, type, frame, total, lost):
        # if self.lines is not None and self.dyna_lines is not None:
        #        dyn_lines.write('line_id, veh_id, type, frame, total\n')
        if lost:
            self.queue_count.write(f'{queue_id},{veh_id},{type},{frame},{total},Exited\n')
        else:
            self.queue_count.write(f'{queue_id},{veh_id},{type},{frame},{total},Entered\n')

    def close_files(self):
        self.vehicles.close()
        self.tracks.close()
        self.videoinfo.close()
        if self.dyna_line is not None:
            self.dyna_line.close()
        if self.line_count is not None:
            self.line_count.close()
        if self.dyna_queue is not None:
            self.dyna_queue.close()
        if self.queue_count is not None:
            self.queue_count.close()
