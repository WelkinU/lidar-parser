import numpy as np
from collections import deque
from readers import PCAPReader

class VelodyneCalibration:
    '''Loads velodyne calibration info, and precomputes parameters used in lidar decoding'''

    def __init__(self):
        self.calibration_dict = {}

    def get_raw_calibration(self, lidar_type):
        if lidar_type in [34, 'VLP-16']:
            return {
                'lidar_name': 'VLP-16',
                'distCorrection': [0] * 32,
                'rotCorrection': [0] * 32,
                #NOTE: WE'RE CURRENTLY NOT USING vertCorrection, AND THE NUMBERS ARE WRONG
                #SEE PAGE 54-55 FOR CORRECT vertCorrection:
                #https://velodynelidar.com/wp-content/uploads/2019/12/63-9243-Rev-E-VLP-16-User-Manual.pdf
                'vertCorrection': [
                    -10.0, 0.667, -8.667, 2.0, -7.333, 3.333, -6.0, 4.667, 
                    -4.667, 6.0, -3.333, 7.333, -2.0, 8.667, -0.667, 10.0, 
                    -10.0, 0.667, -8.667, 2.0, -7.333, 3.333, -6.0, 4.667, 
                    -4.667, 6.0, -3.333, 7.333, -2.0, 8.667, -0.667, 10.0],
                'vertAngle': [-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, 
                    -5, 11, -3, 13, -1, 15] * 2,
                'distScalar': 1/500,
                'parse_row_len': 300,
            }

        elif lidar_type in [40, 'VLP-32c']:
            #manual: https://icave2.cse.buffalo.edu/resources/sensor-modeling/VLP32CManual.pdf
            return {
                'lidar_name': 'VLP-32c',
                'distCorrection': [0] * 32,
                'rotCorrection': [
                    -1.4, 4.2, -1.4, 1.4, -1.4, 1.4, -4.2, 1.4, -1.4, 4.2, 
                    -1.4, 1.4, -4.2, 1.4, -4.2, 1.4, -1.4, 4.2, -1.4, 4.2, 
                    -4.2, 1.4, -1.4, 1.4, -1.4, 1.4, -1.4, 4.2, -4.2, 1.4, 
                    -1.4, 1.4],
                'vertAngle': [
                    -25.0, -1.0, -1.667, -15.639, -11.31, 0.0, -0.667, -8.843,
                    -7.254, 0.333, -0.333, -6.148, -5.333, 1.333, 0.667, -4.0,
                    -4.667, 1.667, 1.0, -3.667, -3.333, 3.333, 2.333, -2.667, 
                    -3.0, 7.0, 4.667, -2.333, -2.0, 15.0, 10.333, -1.333],
                'distScalar': 1/250,
                'parse_row_len': 300,
            }

        elif lidar_type in [161, 128, 'VLS-128']:
            #manual: https://www.manualslib.com/manual/2351432/Velodyne-Vls-128.html?
            return {
                'lidar_name': 'VLS-128',
                'distCorrection': [0] * 128,
                'rotCorrection': [-6.354, -4.548, -2.732, -0.911, 0.911, 2.732, 4.548, 6.354] * 16,
                'vertAngle': [
                    -11.742, -1.99, 3.4, -5.29, -0.78, 4.61, -4.08, 1.31, -6.5, 
                    -1.11, 4.28, -4.41, 0.1, 6.48, -3.2, 2.19, -3.86, 1.53, 
                    -9.244, -1.77, 2.74, -5.95, -0.56, 4.83, -2.98, 2.41, -6.28, 
                    -0.89, 3.62, -5.07, 0.32, 7.58, -0.34, 5.18, -3.64, 1.75, 
                    -25.0, -2.43, 2.96, -5.73, 0.54, 9.7, -2.76, 2.63, -7.65, 
                    -1.55, 3.84, -4.85, 3.18, -5.51, -0.12, 5.73, -4.3, 1.09, 
                    -16.042, -2.21, 4.06, -4.63, 0.76, 15.0, -3.42, 1.97, -6.85, 
                    -1.33, -5.62, -0.23, 5.43, -3.53, 0.98, -19.582, -2.32, 3.07, 
                    -4.74, 0.65, 11.75, -2.65, 1.86, -7.15, -1.44, 3.95, -2.1, 
                    3.29, -5.4, -0.01, 4.5, -4.19, 1.2, -13.565, -1.22, 4.17, -4.52,
                    0.87, 6.08, -3.31, 2.08, -6.65, 1.42, -10.346, -1.88, 3.51, 
                    -6.06, -0.67, 4.72, -3.97, 2.3, -6.39, -1.0, 4.39, -5.18, 0.21, 
                    6.98, -3.09, 4.98, -3.75, 1.64, -8.352, -2.54, 2.85, -5.84, 
                    -0.45,8.43, -2.87, 2.52, -6.17, -1.66, 3.73, -4.96, 0.43],
                'distScalar': 1/250,
                'parse_row_len': 100,
            }
        
        elif lidar_type in [64, 'HDL-64']:
            #manual: https://www.termocam.it/pdf/manuale-HDL-64E.pdf
            return {
                'lidar_name': 'HDL-64',
                'distCorrection': [
                    124.33361, 140.20363, 136.91667, 138.20892, 127.36885, 137.24095,
                    127.29668, 137.98041, 128.35889, 138.57718, 126.63253, 138.27058,
                    132.7093, 142.7446, 136.26175, 134.2074, 125.87488, 139.21695, 
                    131.76974, 129.01468, 126.22684, 142.89648, 128.43475, 140.12968, 
                    133.81041, 142.90254, 136.89133, 140.72774, 134.26199, 140.66241, 
                    135.88882, 144.5766, 126.00303, 131.30705, 144.90154, 132.62471, 
                    136.01581, 119.9929, 139.6333, 126.3406, 135.23636, 125.9419, 
                    139.68582, 125.86839, 144.06204, 127.85461, 141.89546, 125.04743, 
                    120.37386, 127.21017, 120.92606, 120.11496, 137.67897, 131.48471, 
                    142.67419, 126.40719, 145.47238, 131.66919, 146.41878, 128.31795, 
                    143.22624, 129.64432, 143.05737, 132.14954], 
                'rotCorrection': [
                    -5.0845051, -3.0250564, 2.709583, 4.9439402, -0.79002732, 1.4516417, 
                    -1.5833588, 0.67770439, 3.48964, 5.7532492, 2.7112179, 4.9685044, 
                    -5.0960631, -2.853025, -5.8788915, -3.661587, -0.83376622, 1.4524519, 
                    -1.6249346, 0.63937187, 3.435133, 5.6596255, 2.6606722, 4.8981304, 
                    -5.1491637, -2.8820028, -5.9032283, -3.6671956, -0.86227751, 1.3837074, 
                    -1.6624222, 0.59294689, -8.1883736, -4.5925603, 4.4316654, 7.5564623, 
                    -1.1213841, 2.3688171, -2.4531522, 1.073143, 5.7423124, 9.1913319, 
                    4.6058369, 7.9588904, -7.9156561, -4.3973765, -9.2230768, -5.7551842,
                    -1.1089242, 2.3031151, -2.3697047, 1.0869392, 5.5590072, 8.8733768, 
                    4.4075007, 7.8092132, -7.5927958, -4.2950397, -9.0147734, -5.5843472,
                    -1.113239, 2.0938871, -2.293185, 1.0283772],
                'vertAngle': [
                    -6.9228721, -6.5601802, 0.515571, 0.86454099, -6.2895489, -5.909729,
                    -8.2875338, -7.9247899, -5.5943599, -5.2159338, -7.6350541, -7.2711792,
                    -2.8761749, -2.4998751, -4.8510051, -4.5368962, -2.1958711, -1.8215441,
                    -4.2527981, -3.8565781, -1.505584, -1.145044, -3.558001, -3.177505, 
                    1.243884, 1.541196, -0.79144001, -0.461198, 1.861825, 2.208066, -0.177774,
                    0.199356, -22.247231, -21.903254, -11.378545, -10.604312, -21.547476, 
                    -21.008484, -24.584133, -24.055059, -20.46557, -19.953981, -23.452623, 
                    -23.069069, -16.191755, -15.8325, -19.334839, -18.780279, -15.374008, 
                    -14.891615, -18.382517, -17.951117, -14.312524, -13.697269, -17.387566, 
                    -16.83769, -10.057948, -9.6941872, -13.149875, -12.762784, -9.2201195, 
                    -8.808835, -12.260476, -11.769718],
                'distScalar': 1/250,
                'parse_row_len': 300,
                }

        elif lidar_type in [32, 'HDL-32E']:
            #Calibration for Velodyne-32e lidar may be invalid (need to validate rotCorrection).
            #Check to make sure results make sense!

            return {
                'lidar_name': 'HDL-32E',
                'distCorrection': [0] * 32,
                'rotCorrection': [
                    -1.4, 4.2, -1.4, 1.4, -1.4, 1.4, -4.2, 1.4, -1.4, 4.2, 
                    -1.4, 1.4, -4.2, 1.4, -4.2, 1.4, -1.4, 4.2, -1.4, 4.2, 
                    -4.2, 1.4, -1.4, 1.4, -1.4, 1.4, -1.4, 4.2, -4.2, 1.4, 
                    -1.4, 1.4],

                # from manual page 13 https://www.termocam.it/pdf/manuale-HDL-32E.pdf
                'vertAngle': [30.67, -9.33, -29.33, -8.00, -28.00, -6.66, -26.66,
                            -5.33, -25.33, -4.00, -24.00, -2.67, -22.67, -1.33,
                            -21.33, 0.00, -20.00, 1.33, -18.67, 2.67, -17.33, 4.00,
                            -16.00, 5.33, -14.67, 6.67, -13.33, 8.00, -12.00, 9.33,
                            -10.67, 10.67],
                'distScalar': 1/500,
                'parse_row_len': 300,
                'default_frames_per_rotation': 150,
            }
        
        else:
            raise ValueError(f'Lidar type {lidar_type} does not have calibration info to load!')

    def precompute_lidar_params(self, lidar_type):
        ret = self.get_raw_calibration(lidar_type)

        #------------------------------------------
        # List -> numpy, then precompute some stuff
        #------------------------------------------
        
        for key in ret:
            if isinstance(ret[key], list):
                ret[key] = np.array(ret[key], dtype = np.float32)

        ret['vertAngle'] *= np.pi/180
        ret['elevationAngleRadiansCos'] = np.cos(ret['vertAngle'])
        ret['elevationAngleRadiansSin'] = np.sin(ret['vertAngle'])
        ret.pop('vertAngle') #we don't need this any more, remove from dict

        ret['distCorrection'] *= 0.01 #only needed for HDL-64, other lidar's values are all 0
        
        #-----------------------------------------
        # Tile arrays for fast parallel processing
        #-----------------------------------------
        
        if lidar_type in [64, 'HDL-64', 40, 232, 'VLP-32c', 34, 216, 'VLP-16']:
            len_multiplier = 4
        elif lidar_type in [161, 128, 'VLS-128']:
            len_multiplier = 3
        else:
            raise ValueError(f'Lidar type {lidar_type} not supported!')

        for key in ret:
            if isinstance(ret[key], np.ndarray):
                ret[key] = np.tile(ret[key], len_multiplier)

        return ret

    def get_calibration(self, lidar_type):
        if lidar_type not in self.calibration_dict:
            self.calibration_dict[lidar_type] = self.precompute_lidar_params(lidar_type)

        return self.calibration_dict[lidar_type]

class VelodynePacketParser:

    def __init__(self, framedata: bytes, endian: str, cal: VelodyneCalibration):
        #productID - See table on page 57: https://velodynelidar.com/wp-content/uploads/2019/12/63-9243-Rev-E-VLP-16-User-Manual.pdf 
        self.productID = int.from_bytes(framedata[-1:], endian, signed = False)
        self.cal = cal.get_calibration(self.productID)
        headerSize = 42 #header size is 42 for all the supported velodyne models
        
        self.lidardata = framedata[headerSize: 1206 + headerSize]
        self.lidarTimestamp = int.from_bytes(self.lidardata[1200:1204], endian, signed = False)
    
    def decode(self):
        row_len = self.cal['parse_row_len']
        data_len = 100

        ''' We take in frame of 1206 bytes, throw out last 6, then reshape into M x row_len array
        For each row of row_len bytes, truncate down to data_len bytes,
        then we parse rotation, distance, intensity with array slicing:
        
        byte 2,3                ---> rotation azimuth
        bytes 4 + 3N and 5 + 3N ---> distance in meters  (arr[:,4:data_len:3] and arr[:,5:data_len:3])
        bytes 6 + 3N            ---> intensity           (arr[:,6:data_len:3]) '''
        
        arr = np.frombuffer(self.lidardata, dtype = np.uint8)[:1200].astype(np.int32).reshape(-1,row_len)
        rotationAzimuth = ((arr[:,2] + arr[:,3].astype(np.float32) * 256) / 100).reshape(-1).repeat(32)
        distanceInMeters = (arr[:,4:data_len:3] + (arr[:,5:data_len:3] * 256)).reshape(-1) * self.cal['distScalar']
        intensity = arr[:,6:data_len:3].reshape(-1)

        #Apply rotation and distance corrections
        rotationAzimuth -= self.cal['rotCorrection']
        rotationAzimuth *= np.pi / 180 #deg to radians
        distanceInMeters += self.cal['distCorrection']

        # Convert to cartesian coords
        x = distanceInMeters * np.sin(rotationAzimuth) * self.cal['elevationAngleRadiansCos']
        y = distanceInMeters * np.cos(rotationAzimuth) * self.cal['elevationAngleRadiansCos']
        z = distanceInMeters * self.cal['elevationAngleRadiansSin']

        point_cloud = np.transpose(np.stack([x,y,z,intensity])) #TODO: Is it faster to preallocate this array?    
        
        #save pie slice angles in the object
        self.startAngle = rotationAzimuth[0]

        return point_cloud

class VelodyneReader:

    def __init__(self, reader:object = PCAPReader, MAX_FRAMES_PER_ROTATION:int = 1200):
        self.reader = reader
        self.MAX_FRAMES_PER_ROTATION = MAX_FRAMES_PER_ROTATION
        self.cal = VelodyneCalibration()
        self.lidar_name = 'need to read 1 frame of lidar before we have lidar_name'

    def __iter__(self):
        ANGLE_TOLERANCE = 0.01
        data_queue = deque(maxlen = self.MAX_FRAMES_PER_ROTATION)
        angle_queue = deque(maxlen = self.MAX_FRAMES_PER_ROTATION)

        wraparound_detect = False

        for data in self.reader:
            if len(data) != 1248: #not a lidar data packet
                continue

            assert self.reader.endian in ['little', 'big']
            velo = VelodynePacketParser(data, self.reader.endian, self.cal)

            self.lidar_name = velo.cal['lidar_name']
            data_queue.append(velo.decode())
            angle_queue.append(np.degrees(velo.startAngle))

            if len(angle_queue) >= 2 and angle_queue[-2] - angle_queue[-1] > 0:
                wraparound_detect = True

            # autodetect full point cloud - at least 20 frames, includes a wraparound, and last angle > first angle
            if len(angle_queue) > 20 and wraparound_detect and angle_queue[-1] + ANGLE_TOLERANCE > angle_queue[0]:
                self.frames_per_rotation = len(angle_queue) #varies up and down, spec is within 3% of true frames per rotation
                yield np.vstack(data_queue)
                data_queue.clear()
                angle_queue.clear()
                wraparound_detect = False
