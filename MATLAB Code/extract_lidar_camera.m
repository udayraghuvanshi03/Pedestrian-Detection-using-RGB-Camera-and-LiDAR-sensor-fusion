clear all
dataPath = '/home/uday/catkin_ws/src/Final_project/day.bag';
bag = rosbag(dataPath);

camera = select(bag,'Topic','/camera_array/cam0/image_raw');
lidar = select(bag,'Topic','/ns1/velodyne_points');
ir = select(bag,'Topic','/boson_camera_array/cam_right/image_raw');


cameraMsgs = readMessages(camera);
lidarMsgs = readMessages(lidar);
irMsgs = readMessages(ir);

ts1 = timeseries(camera);
ts2 = timeseries(lidar);
ts3 = timeseries(ir);

t1 = ts1.Time;
t2 = ts2.Time;
t3 = ts3.Time;

k = 1;
k2 = 1;

if size(t2,1) > size(t1,1)
    for i = 1:size(t1,1)
        [val,indx] = min(abs(t1(i) - t2))
        if val <= 0.1
            idx(k,:) = [i indx];
            k = k + 1;
        end
    end
else
    for i = 1:size(t2,1)
        [val,indx] = min(abs(t2(i) - t1));
        if val <= 0.1
            idx(k,:) = [indx i];
            k = k + 1;
        end
    end
end

if size(t3,1) > size(t1,1)
    for j = 1:size(t1,1)
        [val2,indx2] = min(abs(t1(j) - t3));
        if val2 <= 0.1
            idx2(k2,:) = [j indx2];
            k2 = k2 + 1;
        end
    end
else
    for j = 1:size(t3,1)
        [val2,indx2] = min(abs(t3(j) - t1));
        if val2 <= 0.1
            idx2(k2,:) = [indx2 j];
            k2 = k2 + 1;
        end
    end
end

% pcFilesPath = '/home/uday/catkin_ws/src/Final_project/pointcloud_data';
% imageFilesPath = '/home/uday/catkin_ws/src/Final_project/camera_data';
% irFilesPath = '/home/uday/catkin_ws/src/Final_project/ir_data';
% 
% if ~exist(imageFilesPath,'dir')
%     mkdir(imageFilesPath);
% end
% if ~exist(pcFilesPath,'dir')
%     mkdir(pcFilesPath);
% end
% if ~exist(irFilesPath,'dir')
%     mkdir(irFilesPath);
% end
% 
% for i = 1:length(idx)
%     RGB = readImage(cameraMsgs{idx(i,1)});
%     pc = pointCloud(readXYZ(lidarMsgs{idx(i,2)}));
%     IR = readImage(irMsgs{idx2(i,2)});
% 
%     n_strPadded = sprintf('%04d',i) ;
%     pcFileName = strcat(pcFilesPath,'/',n_strPadded,'.pcd');
%     imageFileName = strcat(imageFilesPath,'/',n_strPadded,'.jpg');
%     irFileName = strcat(irFilesPath,'/',n_strPadded,'.jpg');
% 
%     imwrite(RGB,imageFileName);
%     pcwrite(pc,pcFileName);
%     imwrite(IR,irFileName);
% end
% 
% 
% 
% 
% 



