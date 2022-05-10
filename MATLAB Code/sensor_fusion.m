clear all
rgbimg ='/home/uday/catkin_ws/src/Final_project/camera_data/0393.jpg';
ptCloud = '/home/uday/catkin_ws/src/Final_project/pointcloud_data/0393.pcd';
% irimg='/home/uday/catkin_ws/src/Final_project/ir_data/0203.jpg';
dataPath = '/home/uday/catkin_ws/src/Final_project/day.bag';
bag = rosbag(dataPath);

pc = pcread(ptCloud);
w = 1224;
h = 1024;
%640X512
%1224X1024

% IR Intrinsics
focalLength_IR = 2*[5.24888150200348832e+02, 5.21776791343664968e+02];
principalPoint_IR = 2*[3.25596989785447420e+02, 3.25596989785447420e+02];
%RGB Intrinsics
focalLength_RGB = [1888.4451558202136, 1888.4000949073984];
principalPoint_RGB = [613.1897651359767, 482.1189409211585];

imageSize = [h, w];
intrinsics = cameraIntrinsics(focalLength_RGB,principalPoint_RGB,imageSize);

%Transformations
frames = bag.AvailableFrames;
gettf= getTransform(bag,frames{21},frames{4});% cam_0_optical_frame and vlp_16 port (frame 18,os_sensor for Ouster) 
tf = gettf.Transform;
ros_quat = tf.Rotation;
quat = [ros_quat.X ros_quat.Y ros_quat.Z ros_quat.W];
rotm = quat2rotm(quat);

ros_trans = tf.Translation;
trans_RGB = [ros_trans.X ros_trans.Y ros_trans.Z];
trans_IR = [-0.8 -0.23 0];
tform = rigid3d(rotm, trans_RGB); %RGB transformation matrix
% tform=rigid3d(rotm,trans_IR);% IR transformation matrix


imPts = projectLidarPointsOnImage(pc,intrinsics,tform);
figure(1)
imshow(rgbimg)
%  I=imread(rgbimg);
%plot(imPts(:,1),imPts(:,2),'.','Color','r')

figure(2)
pcshow(pc)

labels_loc = '/home/uday/catkin_ws/src/yolov5/runs/detect/exp4/labels/393.txt';
fileID = fopen(labels_loc,'r');
formatSpec = '%f';
A = fscanf(fileID,formatSpec);
w = 1224;
h = 1024;
k = 1;
clusterpoints = [];
bboxLidar_dist=[];
if length(A)>=5
    for i = 1:5:length(A)
        if A(i)==0
            x_center= A(i+1,1)*w;
            y_center= A(i+2,1)*h;
            width= A(i+3,1)*w;
            height= A(i+4,1)*h;
            xLeft = x_center - width/2;
            yLeft = y_center - width/2;
            xBottom = x_center - height/2;
            yBottom = y_center - height/2;
            bbox = [xLeft, yBottom, width, height];
            figure(1)
            hold on 
            
            
            for j = 1:length(imPts)
                if imPts(j,1)<=xLeft+width &&  imPts(j,1)>=xLeft
                    if imPts(j,2)<=yBottom+height &&  imPts(j,2)>=yBottom
                        clusterpoints(k, 1) = imPts(j,1);
                        clusterpoints(k, 2) = imPts(j,2);
                        k = k+1;
                    end
                end
            end
            
            
            rectangle('Position', bbox, 'EdgeColor', 'b', 'LineWidth', 2);
           

            if isempty(clusterpoints) == false
                plot(clusterpoints(:,1),clusterpoints(:,2),'.','Color','r')
            end
            
            bboxLidar = bboxCameraToLidar(bbox,pc,intrinsics,invert(tform),'ClusterThreshold',1);
            bbox=[floor(xLeft), floor(yBottom), floor(width), floor(height)];
            dist_x=bboxLidar(:,1);
            dist_y=bboxLidar(:,2);
            dist_z=bboxLidar(:,3);
            if isempty(dist_x)==false
                dist=sqrt(dist_x^2+dist_y^2+dist_z^2)-2;
                labels="pedestrian"+dist+"m";
                figure(1)
                hold on 
                showShape('rectangle',bbox,'color','yellow','Label',labels)

                figure(2)
                hold on
                showShape('cuboid',bboxLidar,'Opacity',0.5,'Color','red', ...
                    'Label',labels,'LabelTextColor','green','LabelFontSize',7,'LabelOpacity',0.3)
                
            end
%             figure(2)
%             hold on
%             showShape('cuboid',bboxLidar,'Opacity',0.5,'Color','red')

        end
    end
end

hold off


