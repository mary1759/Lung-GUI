function varargout = LungGUI(varargin)
% LUNGGUI MATLAB code for LungGUI.fig
%      LUNGGUI, by itself, creates a new LUNGGUI or raises the existing
%      singleton*.
%
%      H = LUNGGUI returns the handle to a new LUNGGUI or the handle to
%      the existing singleton*.
%
%      LUNGGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in LUNGGUI.M with the given input arguments.
%
%      LUNGGUI('Property','Value',...) creates a new LUNGGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before LungGUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to LungGUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help LungGUI

% Last Modified by GUIDE v2.5 05-Dec-2019 15:31:14

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @LungGUI_OpeningFcn, ...
    'gui_OutputFcn',  @LungGUI_OutputFcn, ...
    'gui_LayoutFcn',  [] , ...
    'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before LungGUI is made visible.
function LungGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to LungGUI (see VARARGIN)

% Choose default command line output for LungGUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes LungGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = LungGUI_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function leftlungtitlebar_Callback(hObject, eventdata, handles)
% hObject    handle to leftlungtitlebar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of leftlungtitlebar as text
%        str2double(get(hObject,'String')) returns contents of leftlungtitlebar as a double


% --- Executes during object creation, after setting all properties.
function leftlungtitlebar_CreateFcn(hObject, eventdata, handles)
% hObject    handle to leftlungtitlebar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function rightlungtitlebar_Callback(hObject, eventdata, handles)
% hObject    handle to rightlungtitlebar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of rightlungtitlebar as text
%        str2double(get(hObject,'String')) returns contents of rightlungtitlebar as a double


% --- Executes during object creation, after setting all properties.
function rightlungtitlebar_CreateFcn(hObject, eventdata, handles)
% hObject    handle to rightlungtitlebar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in loadimage.
function loadimage_Callback(hObject, eventdata, handles)
% hObject    handle to loadimage (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles = guidata(hObject);

cla(handles.axes1,'reset');
cla(handles.axes2,'reset');

%clc;    % Clear the command window.
%close all;  % Close all figures (except those of imtool.)
%imtool close all;  % Close all imtool figures if you have the Image Processing Toolbox.
%clear;  % Erase all existing variables. Or clearvars if you want.
%workspace;  % Make sure the workspace panel is showing.
format long g;
format compact;
fontSize = 20;
global images;
global ii;
global nfiles;
global RightLungYinter;
global LeftLungYinter;
% Read in a standard MATLAB gray scale demo image.
rootpath = 'C:\Users\CADLAB-4\Desktop\Fall19\MII\MatlabProject\CT images';
%file_contente = {};
% Read in a standard MATLAB gray scale demo image.
%rootpath = pwd;
%file_contente = {};

% [baseFileName,path] = uigetfile('*.dcm', rootpath, 'MultiSelect', 'on');
%
% % Get the full filename, with path prepended.
% fullFileName = fullfile(path, baseFileName);
% % Check if file exists.
% if ~exist(fullFileName, 'file')
% 	% File doesn't exist -- didn't find it there.  Check the search path for it.
% 	fullFileName = baseFileName; % No path this time.
% 	if ~exist(fullFileName, 'file')
% 		% Still didn't find it.  Alert user.
% 		errorMessage = sprintf('Error: %s does not exist in the search path folders.', fullFileName);
% 		uiwait(warndlg(errorMessage));
% 		return;
% 	end
% end

imagefiles = dir('*.dcm');

nfiles = length(imagefiles);    % Number of files found

for ii=1:nfiles
    currentfilename = imagefiles(ii).name;
    currentimage = dicomread(currentfilename);
    currentimageinfo = dicominfo(currentfilename);
    img_loc = currentimageinfo.InstanceNumber;
    % currentimage = currentimage + currentimageinfo.RescaleIntercept; % HU
    currentimage = currentimage;
    images{img_loc} = currentimage;
end

for ii=1:nfiles
    imshow(images{ii},[], 'InitialMagnification','fit','Parent',handles.axes1);
    sliderstr = sprintf('current slice: %d of %d', ii, nfiles);
    set(handles.sliderTxt, 'string', sliderstr);
    pause(0.1)
end

global segFlg;
segFlg = 0;


% grayImage = dicomread(fullFileName);
% grayImage = double(grayImage);
%figure, imshow(grayImage, [-1900, 350]); %fig1
% imshow(grayImage,[], 'InitialMagnification','fit','Parent',handles.axes1);
% impixelinfo();

%handles.grayImage = grayImage;
%handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% --- Executes on button press in segmentleftrightlung.
function segmentleftrightlung_Callback(hObject, eventdata, handles)
% hObject    handle to segmentleftrightlung (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Threshold the image to binarize it.
handles = guidata(hObject);
global images;
% global ii;
global nfiles;
global leftlungvolumefinal;
global rightlungvolumefinal;
global totallungvolumefinal;
global finalImageRightRow;
global finalImageLeftRow;
global segFlg;
global segImgs;
global finalImage_rightlung;
global finalImage_leftlung;

cla(handles.axes1,'reset');
cla(handles.axes2,'reset');
% % imagefiles = dir('*.dcm');
% midLoc = round(nfiles/2);
% % seg midLoc slice
% segImg = images{midLoc};
% bw = segImg < -750;
% % figure, imshow(bw,[])
% bw2 = bwareafilt(bw,3);
% % figure, imshow(bw2,[])
% segImg = bw2 - bwareafilt(bw2, 1);
% % figure, imshow(segImg,[])
% segImg = imclose(segImg, strel('disk',3));
% segImg = imfill(segImg, 'holes');
% segImg = logical(segImg);
% %figure(1), imshow(segImg,[]), title(sprintf('slice number: %d',midLoc))
% % centroids = regionprops(segImg,'centroid');
% % centroids = [centroids.Centroid];
% % centroid1 = round(centroids(1:2));
% % centroid2 = round(centroids(3:4));
%
%
% % origcentroid1 = centroid1;
% % origcentroid2 = centroid2;
%
% segImgs = cell(1,nfiles);
% segImgs(1,:) = {zeros(size(segImg))};
% segImgs{midLoc} = cast(segImg, class(images{midLoc})).*images{midLoc};
%
%
% for val = midLoc+1:nfiles
%     currImg = images{val};
%     bw = currImg < -750;
%     % figure, imshow(bw,[])
%     bw = logical(bw);
%     bw = bwlabel(bw);
%     % figure, imshow(bw,[])
%     labels = [bw(8,8), bw(500,8)];
%     bw2 = ismember(bw, labels);
%     bw = bw-bw2;
%
%     bw = bw - bwareafilt(logical(bw), 1);
%     bw = logical(bw);
%     % figure, imshow(bw,[])
%
%     bw = bwareafilt(logical(bw),2);
%     segImg = imclose(bw, strel('disk',4));
%     segImg = imfill(segImg, 'holes');
%     segImg = logical(segImg);
%     %%
%     label_img = bwlabel(segImg);
%     mainImg_idxs = find(segImgs{val-1});
%     CC = bwconncomp(segImg);
%     subBlobs_idxs = CC.PixelIdxList;
%     keepBlob = double.empty();
%     TOT_OBJS = CC.NumObjects;
%     for obj = 1:TOT_OBJS
%         truth = any(ismember(cell2mat(subBlobs_idxs(1,obj)),mainImg_idxs));
%         if truth
%             keepBlob(end+1) =  obj;
%         end
%     end
%     keepImg = ismember(label_img,keepBlob);
%     segImgs{val} = cast(keepImg, class(images{val})).*images{val};
%     % figure(1), imshow( segImgs{val},[]), title(sprintf('slice number: %d',val))
% end
%
% for val = midLoc-1:-1:1
%     currImg = images{val};
%     bw = currImg < -750;
%     % figure, imshow(bw,[])
%     bw = logical(bw);
%     bw = bwlabel(bw);
%     % figure, imshow(bw,[])
%     labels = [bw(8,8), bw(500,8)];
%     bw2 = ismember(bw, labels);
%     bw = bw-bw2;
%
%     bw = bw - bwareafilt(logical(bw), 1);
%     bw = logical(bw);
%     % figure, imshow(bw,[])
%
%     bw = bwareafilt(logical(bw),2);
%     segImg = imclose(bw, strel('disk',4));
%     segImg = imfill(segImg, 'holes');
%     segImg = logical(segImg);
%     %%
%     label_img = bwlabel(segImg);
%     mainImg_idxs = find(segImgs{val+1});
%     CC = bwconncomp(segImg);
%     subBlobs_idxs = CC.PixelIdxList;
%     keepBlob = double.empty();
%     TOT_OBJS = CC.NumObjects;
%     for obj = 1:TOT_OBJS
%         truth = any(ismember(cell2mat(subBlobs_idxs(1,obj)),mainImg_idxs));
%         if truth
%             keepBlob(end+1) =  obj;
%         end
%     end
%     keepImg = ismember(label_img,keepBlob);
%     segImgs{val} = cast(keepImg, class(images{val})).*images{val};
%     % figure(1), imshow( segImgs{val},[]), title(sprintf('slice number: %d',val))
% end
%
%
% for val = 1:nfiles
%     imshow(images{val},[], 'InitialMagnification','fit','Parent',handles.axes1);
%     imshow(segImgs{val},[], 'InitialMagnification','fit','Parent',handles.axes2);
%     pause(0.01)
%     sliderstr = sprintf('current slice: %d of %d', val, nfiles);
%     set(handles.sliderTxt, 'string', sliderstr);
% end
%
% segFlg = 1;
%




%%

global RightLungYinter
global LeftLungYinter

for ii=1:nfiles
    grayImage = double(images{ii});
    grayImage2 = grayImage;
    binaryImage =  grayImage > 600; %-250
    binaryImage = imfill(binaryImage, 'holes');
    numberToExtract = 1;
    biggestBlob = bwareafilt(binaryImage, numberToExtract);
    
    initial_image = grayImage .* double(biggestBlob);
    % figure, imshow(initial_image, []); %fig3
    % impixelinfo();
    % Threshold the image to binarize it.
    
    binaryImage2 = initial_image < 600;
    binaryImage3 = initial_image > 100;
    
    matchingPixels{ii} = (double(binaryImage2) - double(binaryImage3)) == 0;
    %imshow(matchingPixels);
    %figure, imshow(matchingPixels, []); %fig 4
    %impixelinfo();
    % Fill holes
    matchingPixels{ii} = imfill(matchingPixels{ii}, 'holes');
    %figure, imshow(matchingPixels, []);
    % Ask user how many blobs to extract.
    %numberToExtract = 1;
    % regionprops
    stats = regionprops(matchingPixels{ii},'Area');
    areas = [stats.Area];
    areas = sort(areas, 'descend');
    % Ask user how many blobs to extract.
    % numberToExtract = 1;
    % Only left lung segmentation
    
    if ismember(ii, [1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 134 135 136 137 138 139 140])
        leftlungvolumefinal{ii} = 0;
        rightlungvolumefinal{ii} = 0;
        biggestBlob2{ii} = bwareafilt(matchingPixels{ii}, [areas(2)+100 areas(1)]);
        %leftlungvolumefinal{ii} = 1.68227944336 * areas(2);
        %rightlungvolumefinal{ii} = 1.68227944336 * areas(1);
        biggestBlob2{ii} = double(biggestBlob2{ii});
        %figure, imshow(matchingPixels, []);
        finalImage_leftlung{ii} = initial_image .* biggestBlob2{ii};
        imshow(finalImage_leftlung{ii}, 'InitialMagnification','fit','Parent',handles.axes1);
        impixelinfo();
        biggestBlob3{ii} = bwareafilt(matchingPixels{ii}, [areas(2) areas(1)-100]);
        %leftlungvolumefinal{ii} = 1.68227944336 * areas(2);
        biggestBlob3{ii} = double(biggestBlob3{ii});
        finalImage_rightlung{ii} = grayImage2.*biggestBlob3{ii};
        imshow(finalImage_rightlung{ii}, 'InitialMagnification','fit','Parent',handles.axes2);
        impixelinfo();
        
    else
        try
            biggestBlob2{ii} = bwareafilt(matchingPixels{ii}, [areas(2)+100 areas(1)]);
            %leftlungvolumefinal{ii} = 1.68227944336 * areas(2);
            rightlungvolumefinal{ii} = 1.68227944336 * areas(1);
            biggestBlob2{ii} = double(biggestBlob2{ii});
            %figure, imshow(matchingPixels, []);
            finalImage_leftlung{ii} = initial_image .* biggestBlob2{ii};
            biggestBlob3{ii} = bwareafilt(matchingPixels{ii}, [areas(2) areas(1)-100]);
            leftlungvolumefinal{ii} = 1.68227944336 * areas(2);
            biggestBlob3{ii} = double(biggestBlob3{ii});
            finalImage_rightlung{ii} = grayImage2.*biggestBlob3{ii};
            imshow(finalImage_leftlung{ii},[], 'InitialMagnification','fit','Parent',handles.axes1);
            impixelinfo();
            imshow(finalImage_rightlung{ii},[], 'InitialMagnification','fit','Parent',handles.axes2);
            impixelinfo();
        catch
            biggestBlob2{ii} = bwareafilt(matchingPixels{ii}, [areas(2) areas(1)+100]);
            %leftlungvolumefinal{ii} = 1.68227944336 * areas(2);
            rightlungvolumefinal{ii} = 1.68227944336 * areas(1);
            biggestBlob2{ii} = double(biggestBlob2{ii});
            %figure, imshow(matchingPixels, []);
            finalImage_leftlung{ii} = initial_image .* biggestBlob2{ii};
            biggestBlob3{ii} = bwareafilt(matchingPixels{ii}, [areas(2)-100 areas(1)]);
            leftlungvolumefinal{ii} = 1.68227944336 * areas(2);
            biggestBlob3{ii} = double(biggestBlob3{ii});
            finalImage_rightlung{ii} = grayImage2.*biggestBlob3{ii};
            imshow(finalImage_leftlung{ii},[], 'InitialMagnification','fit','Parent',handles.axes2);
            impixelinfo();
            imshow(finalImage_rightlung{ii},[], 'InitialMagnification','fit','Parent',handles.axes1);
            impixelinfo();
        end
    end
    
    totallungvolumefinal{ii} = leftlungvolumefinal{ii}+rightlungvolumefinal{ii};
    
    %     %%
    %     try
    %         if ~ismember(ii, [1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 134 135 136 137 138 139 140])
    %             biggestBlob2{ii} = bwareafilt(matchingPixels{ii}, [areas(2)+100 areas(1)]);
    %             leftlungvolumefinal{ii} = 1.68227944336 * areas(2);
    %             rightlungvolumefinal{ii} = 1.68227944336 * areas(1);
    %
    %             totallungvolumefinal{ii} = leftlungvolumefinal{ii}+rightlungvolumefinal{ii};
    %
    %         else
    %             biggestBlob2{ii} = zeros(512,512);
    %             leftlungvolumefinal{ii}= 0;
    %             rightlungvolumefinal{ii} = 0;
    %             totallungvolumefinal{ii} = 0;
    %         end
    %     catch
    %         biggestBlob2{ii} = zeros(512,512);
    %         leftlungvolumefinal{ii}= 0;
    %         rightlungvolumefinal{ii} = 0;
    %         totallungvolumefinal{ii} = 0;
    %     end
    %
    %
    %     biggestBlob2{ii} = double(biggestBlob2{ii});
    %     %figure, imshow(matchingPixels, []);
    %     finalImage_leftlung{ii} = initial_image .* biggestBlob2{ii};
    %
    %     imshow(finalImage_leftlung{ii},[], 'InitialMagnification','fit','Parent',handles.axes1);
    %     impixelinfo();
    %     %figure, imshow(biggestBlob2, []); %fig 5
    %     %biggestBlob2 = bwareafilt(binaryImage2, numberToExtract);
    %     % figure, imshow(biggestBlob2, []); %fig 5
    %     % impixelinfo();
    %     try
    %         biggestBlob3{ii} = bwareafilt(matchingPixels{ii}, [areas(2) areas(1)-100]);
    %         leftlungvolumefinal{ii} = 1.68227944336 * areas(2);
    %         totallungvolumefinal{ii} = leftlungvolumefinal{ii}+rightlungvolumefinal{ii};
    %     catch
    %         biggestBlob3{ii} = zeros(512,512);
    %         leftlungvolumefinal{ii} = 0;
    %         rightlungvolumefinal{ii} = 0;
    %         totallungvolumefinal{ii} = leftlungvolumefinal{ii}+rightlungvolumefinal{ii};
    %     end
    %
    %     biggestBlob3{ii} = double(biggestBlob3{ii});
    %     finalImage_rightlung{ii} = grayImage2.*biggestBlob3{ii};
    %
    %     %     if ismember(ii, [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,134,135,136,137,138,139,140])
    %     %         imshow(finalImage_leftlung{ii},[], 'InitialMagnification','fit','Parent',handles.axes2);
    %     %     else
    %     %         imshow(finalImage_rightlung{ii},[], 'InitialMagnification','fit','Parent',handles.axes2);
    %     %     end
    %     %     imshow(finalImage_rightlung{ii},[], 'InitialMagnification','fit','Parent',handles.axes2);
    %
    %
    %     if ~ismember(ii, [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,134,135,136,137,138,139,140])
    %         imshow(finalImage_rightlung{ii},[], 'InitialMagnification','fit','Parent',handles.axes2);
    %     else
    %         imshow(finalImage_rightlung{ii})
    %     end
    %     %imshow(finalImage_rightlung{ii},[], 'InitialMagnification','fit','Parent',handles.axes2);
    %
    %     %figure, imshow(finalImage_rightlung{ii}, []);
    %     % finalImage_rightlung{ii} = .*finalImage_rightlung{ii};
    %     %imshow(finalImage_rightlung{ii},[], 'InitialMagnification','fit','Parent',handles.axes2);
    %     %impixelinfo();
    %%
    
    finalImageRightRow{ii} = finalImage_rightlung{ii}(:);
    RightLungYinter{ii} = nonzeros(finalImageRightRow{ii})-1024; %Excluding non zeros
    meanRightLung = mean(RightLungYinter{ii});
    finalImageLeftRow{ii} = finalImage_leftlung{ii}(:);
    LeftLungYinter{ii} = nonzeros(finalImageLeftRow{ii})-1024;
    meanLeftLung = mean(LeftLungYinter{ii});
    finalImageRow = (meanRightLung + meanLeftLung)/2;
    %meanRightLung = mean(nonzeros(finalImageRightRow));
    pause(0.01);
    % voxelvolume = 1.68227944336;
    
    if ismember(ii, [1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 134 135 136 137 138 139 140])
        leftlungvolumefinal{ii} = 0;
        rightlungvolumefinal{ii} = 0;
        totallungvolumefinal{ii} = leftlungvolumefinal{ii}+rightlungvolumefinal{ii};
    end
    
    %%
    set(handles.leftvolumeval, 'string', num2str(leftlungvolumefinal{ii}));
    set(handles.rightvolumeval, 'string', num2str(rightlungvolumefinal{ii}));
    set(handles.totallungvolume, 'string', num2str(totallungvolumefinal{ii}));
    
    set(handles.meanrightlung, 'string', num2str(meanRightLung));
    set(handles.meanleftlung, 'string', num2str(meanLeftLung));
    set(handles.finalimagerow, 'string', num2str(finalImageRow));
    
    
    sliderstr = sprintf('current slice: %d of %d', ii, nfiles);
    set(handles.sliderTxt, 'string', sliderstr);
    
    
end

segFlg = 1;

function leftvolumeval_Callback(hObject, eventdata, handles)
% hObject    handle to leftvolumeval (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of leftvolumeval as text
%        str2double(get(hObject,'String')) returns contents of leftvolumeval as a double


% --- Executes during object creation, after setting all properties.
function leftvolumeval_CreateFcn(hObject, eventdata, handles)
% hObject    handle to leftvolumeval (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function rightvolumeval_Callback(hObject, eventdata, handles)
% hObject    handle to rightvolumeval (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of rightvolumeval as text
%        str2double(get(hObject,'String')) returns contents of rightvolumeval as a double


% --- Executes during object creation, after setting all properties.
function rightvolumeval_CreateFcn(hObject, eventdata, handles)
% hObject    handle to rightvolumeval (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function totallungvolume_Callback(hObject, eventdata, handles)
% hObject    handle to totallungvolume (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of totallungvolume as text
%        str2double(get(hObject,'String')) returns contents of totallungvolume as a double


% --- Executes during object creation, after setting all properties.
function totallungvolume_CreateFcn(hObject, eventdata, handles)
% hObject    handle to totallungvolume (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function meanleftlung_Callback(hObject, eventdata, handles)
% hObject    handle to meanleftlung (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of meanleftlung as text
%        str2double(get(hObject,'String')) returns contents of meanleftlung as a double


% --- Executes during object creation, after setting all properties.
function meanleftlung_CreateFcn(hObject, eventdata, handles)
% hObject    handle to meanleftlung (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function finalimagerow_Callback(hObject, eventdata, handles)
% hObject    handle to finalimagerow (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of finalimagerow as text
%        str2double(get(hObject,'String')) returns contents of finalimagerow as a double


% --- Executes during object creation, after setting all properties.
function finalimagerow_CreateFcn(hObject, eventdata, handles)
% hObject    handle to finalimagerow (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function meanrightlung_Callback(hObject, eventdata, handles)
% hObject    handle to meanrightlung (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of meanrightlung as text
%        str2double(get(hObject,'String')) returns contents of meanrightlung as a double


% --- Executes during object creation, after setting all properties.
function meanrightlung_CreateFcn(hObject, eventdata, handles)
% hObject    handle to meanrightlung (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in AutomaticEmphysemaButton.
function AutomaticEmphysemaButton_Callback(hObject, eventdata, handles)
% hObject    handle to AutomaticEmphysemaButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cla(handles.axes1,'reset');
cla(handles.axes2,'reset');

global finalImageRightRow;
global finalImageLeftRow;
global nfiles;
global finalImage_rightlung;
global finalImage_leftlung;

for ii=70:nfiles
    %% Average operating HU value
    dummy_right = finalImage_rightlung{ii};
    binaryImageRight = dummy_right;
    binaryImageRight(binaryImageRight > 74) = 0;
    %binaryImageRight =
    
    %binaryImageLeft = finalImage_leftlung{ii} < 74;
    finalImage_rightlung{ii} = double(finalImage_rightlung{ii});
    %emphysemaRight{ii} = binaryImageRight .* dummy_right;
    
    [B,~] = bwboundaries(binaryImageRight,'noholes');
    imshow(dummy_right,[], 'InitialMagnification','fit','Parent',handles.axes2);
    visboundaries(handles.axes2,B,'LineWidth',0.5,'Color', 'r')
    %     hold on;
    %
    %     for k = 1:length(B)
    %         boundary = B{k};
    %         plot(boundary(:,2), boundary(:,1), 'r', 'LineWidth', 0.5)
    %     end
    %     hold off;
    
    dummy_left = finalImage_leftlung{ii};
    
    
    binaryImageLeft = dummy_left;
    binaryImageLeft(binaryImageLeft > 74) = 0;
    
    
    
    %binaryImageLeft = finalImage_leftlung{ii} < 74;
    finalImage_leftlung{ii} = double(finalImage_leftlung{ii});
    %emphysemaLeft{ii} = binaryImageLeft .* dummy_left;
    
    [B,~] = bwboundaries(binaryImageLeft,'noholes');
    imshow(dummy_left,[], 'InitialMagnification','fit','Parent',handles.axes1);
    visboundaries(handles.axes1,B,'LineWidth',0.5,'Color', 'g')
    
    %     hold on;
    %
    %     for k1 = 1:length(B)
    %         boundary2 = B{k1};
    %         plot(boundary2(:,2), boundary2(:,1), 'b', 'LineWidth', 0.5)
    %     end
    %     hold off;
    
    %     stats = regionprops('table',emphysemaRight{ii},'Centroid',...
    %         'MajorAxisLength','MinorAxisLength');
    %     centers2 = stats.Centroid;
    %     diameters2 = mean([stats.MajorAxisLength stats.MinorAxisLength],2);
    %     radii2 = diameters2/2;
    %     hold on
    %     viscircles(centers2,radii2);
    %     hold off
    
    %     RightLungYinterMat = reshape(finalImageRightRow{ii},[512,512]);
    %     bw1 = RightLungYinterMat < -950;
    %     if find(bw1)~=0
    %         figure, imshow(bw1, []);
    %         emphysema_right{ii} = bw1 .* RightLungYinterMat;
    %         imshow(emphysema_right{ii},[], 'InitialMagnification','fit','Parent',handles.axes2);
    %         stats = regionprops('table',bw1,'Centroid',...
    %             'MajorAxisLength','MinorAxisLength');
    %         centers1 = stats.Centroid;
    %         diameters1 = mean([stats.MajorAxisLength stats.MinorAxisLength],2);
    %         radii1 = diameters1/2;
    %         hold on
    %         viscircles(centers1,radii1);
    %         hold off
    %     end
    %     LeftLungYinterMat = reshape(finalImageLeftRow{ii},[512,512]);
    %     bw2 = LeftLungYinterMat < -950;
    %     figure, imshow(bw2, []);
    %     emphysema_left{ii} = bw2 .* LeftLungYinterMat;
    %     imshow(emphysema_left{ii},[], 'InitialMagnification','fit','Parent',handles.axes1);
    %     stats = regionprops('table',bw2,'Centroid',...
    %     'MajorAxisLength','MinorAxisLength')
    %     centers2 = stats.Centroid;
    %     diameters2 = mean([stats.MajorAxisLength stats.MinorAxisLength],2);
    %     radii2 = diameters2/2;
    %     hold on
    %     viscircles(centers2,radii1);
    %     hold off
    sliderstr = sprintf('current slice: %d of %d', ii, nfiles);
    set(handles.sliderTxt, 'string', sliderstr);
    pause(0.05);
end


% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
global nfiles;
maxNumberOfImages = nfiles;
set(hObject, 'Min', 0);
set(hObject, 'Max', maxNumberOfImages);
set(hObject, 'SliderStep', [1/maxNumberOfImages , 10/maxNumberOfImages ]);

value = int32(get(hObject,'Value'));
if value == 0 || maxNumberOfImages == 1
    value = 1;
elseif value > maxNumberOfImages
    value = maxNumberOfImages;
end
sliderstr = sprintf('current slice: %d of %d', value, maxNumberOfImages);
set(handles.sliderTxt, 'string', sliderstr);

global segFlg
global images
global segImgs




if segFlg == 0 % no seg
    imshow(images{value},[], 'InitialMagnification','fit','Parent',handles.axes1);
    imshow(zeros(size(images{value})),[], 'InitialMagnification','fit','Parent',handles.axes2);
else
    global finalImage_rightlung;
    global finalImage_leftlung;
    
    imshow(finalImage_leftlung{value},[], 'InitialMagnification','fit','Parent',handles.axes1);
    imshow(finalImage_rightlung{value},[], 'InitialMagnification','fit','Parent',handles.axes2);
    
    global leftlungvolumefinal;
    global rightlungvolumefinal;
    
    global totallungvolumefinal
    global RightLungYinter
    global LeftLungYinter
    %% slc
    meanRightLung_slc = mean(RightLungYinter{value});
    meanLeftLung_slc = mean(LeftLungYinter{value});
    totMeanLung_slc = (meanRightLung_slc+meanLeftLung_slc)/2;
    
    volRightLung_slc = rightlungvolumefinal{value};
    volLeftLung_slc = leftlungvolumefinal{value};
    totVolLung_slc = volRightLung_slc+ volLeftLung_slc;
    %% total
    r_vals = RightLungYinter{:};
    t_meanRightLung = mean(r_vals);
    l_vals = LeftLungYinter{:};
    t_meanLefttLung = mean(l_vals);
    t_totMeanLung = (t_meanRightLung+t_meanLefttLung)/2;
    
    l_vals = [leftlungvolumefinal{:}];
    r_vals = [rightlungvolumefinal{:}];
    
    t_volRightLung = sum(r_vals);
    t_volLeftLung = sum(l_vals);
    t_totVolLung = t_volRightLung+ t_volLeftLung;
    %% print
    %sprintf('%0.2f mm^3', t_totVolLung)
    set(handles.leftvolumeval, 'string', sprintf('%0.2f mm^3', volLeftLung_slc));
    
    set(handles.rightvolumeval, 'string', sprintf('%0.2f mm^3', volRightLung_slc));
    set(handles.totallungvolume, 'string', sprintf('%0.2f mm^3', totVolLung_slc));
    
    set(handles.meanrightlung, 'string', num2str(meanRightLung_slc));
    set(handles.meanleftlung, 'string', num2str(meanLeftLung_slc));
    set(handles.finalimagerow, 'string', num2str(totMeanLung_slc));
    
    set(handles.edit12, 'string', sprintf('%0.2f mm^3', t_volLeftLung));
    set(handles.edit13, 'string', sprintf('%0.2f mm^3', t_volRightLung));
    set(handles.edit14, 'string', sprintf('%0.2f mm^3', t_totVolLung));
    
    set(handles.edit15, 'string', num2str(t_meanRightLung));
    set(handles.edit16, 'string', num2str(t_meanLefttLung));
    set(handles.edit17, 'string', num2str(t_totMeanLung));
    
end



% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

format long g;
format compact;
fontSize = 20;
global images;
global ii;
global nfiles;
global RightLungYinter;
global LeftLungYinter;
% Read in a standard MATLAB gray scale demo image.
rootpath = 'C:\Users\CADLAB-4\Desktop\Fall19\MII\MatlabProject\CT images';
%file_contente = {};
% Read in a standard MATLAB gray scale demo image.
%rootpath = pwd;
%file_contente = {};

% [baseFileName,path] = uigetfile('*.dcm', rootpath, 'MultiSelect', 'on');
%
% % Get the full filename, with path prepended.
% fullFileName = fullfile(path, baseFileName);
% % Check if file exists.
% if ~exist(fullFileName, 'file')
% 	% File doesn't exist -- didn't find it there.  Check the search path for it.
% 	fullFileName = baseFileName; % No path this time.
% 	if ~exist(fullFileName, 'file')
% 		% Still didn't find it.  Alert user.
% 		errorMessage = sprintf('Error: %s does not exist in the search path folders.', fullFileName);
% 		uiwait(warndlg(errorMessage));
% 		return;
% 	end
% end

imagefiles = dir('*.dcm');

nfiles = length(imagefiles);    % Number of files found

for ii=1:nfiles
    currentfilename = imagefiles(ii).name;
    currentimage = dicomread(currentfilename);
    currentimageinfo = dicominfo(currentfilename);
    img_loc = currentimageinfo.InstanceNumber;
    % currentimage = currentimage + currentimageinfo.RescaleIntercept; % HU
    currentimage = currentimage;
    images{img_loc} = currentimage;
end

left_regiongrows = cell(1, nfiles);
right_regiongrows = cell(1, nfiles);
for ii=70:80%nfiles
    
    sliderstr = sprintf('current slice: %d of %d', ii, nfiles);
    set(handles.sliderTxt, 'string', sliderstr);
    %% left
    
    
    grayImage = double(images{ii});
    grayImage2 = grayImage;
    binaryImage =  grayImage > 600; %-250
    binaryImage = imfill(binaryImage, 'holes');
    numberToExtract = 1;
    biggestBlob = bwareafilt(binaryImage, numberToExtract);
    
    initial_image = grayImage .* double(biggestBlob);
    % figure, imshow(initial_image, []); %fig3
    % impixelinfo();
    % Threshold the image to binarize it.
    
    binaryImage2 = initial_image < 600;
    binaryImage3 = initial_image > 100;
    
    
    input_img = cast(binaryImage3, class(images{ii})) .* images{ii};
    if ii == 66
        imshow(images{ii},[], 'InitialMagnification','fit','Parent',handles.axes1);
        [y,x] = getpts(handles.axes1);
        x = round(x); y = round(y);
        reg_maxdist = 100;
    else
        ref_img_L = left_regiongrows{ii-1};
        ref_img_R = right_regiongrows{ii-1};
        s = regionprops(logical(ref_img_L),'centroid');
        centroids = round(s.Centroid);
        x(2) = centroids(2);  y(2) = centroids(1);
        
        
        s = regionprops(logical(ref_img_R),'centroid');
        centroids = round(s.Centroid);
        x(1) = centroids(2);  y(1) = centroids(1);
        
        
        
    end
    
    try
        temp_left = regiongrowing(double(input_img), x(2),y(2),reg_maxdist);
        temp_right = regiongrowing(double(input_img), x(1),y(1),reg_maxdist);
        
        
        temp_left = imclose(temp_left, strel('disk', 3));
        temp_right = imclose(temp_right, strel('disk', 3));
        
        temp_left = imfill(temp_left, 'holes');
        temp_right = imfill(temp_right, 'holes');
        
        
        left_regiongrows{ii} = cast(temp_left, class(images{ii})) .* images{ii};
        right_regiongrows{ii} = cast(temp_right, class(images{ii})) .* images{ii};
        
        
    catch
        break
    end
    
    %% right
    
    imshow(left_regiongrows{ii},[], 'InitialMagnification','fit','Parent',handles.axes1);
    imshow(right_regiongrows{ii},[], 'InitialMagnification','fit','Parent',handles.axes2);
    pause(0.5)
end

global segFlg;
segFlg = 0;



guidata(hObject, handles);

function J=regiongrowing(I,x,y,reg_maxdist)
% This function performs "region growing" in an image from a specified
% seedpoint (x,y)
%
% J = regiongrowing(I,x,y,t)
%
% I : input image
% J : logical output image of region
% x,y : the position of the seedpoint (if not given uses function getpts)
% t : maximum intensity distance (defaults to 0.2)
%
% The region is iteratively grown by comparing all unallocated neighbouring pixels to the region.
% The difference between a pixel's intensity value and the region's mean,
% is used as a measure of similarity. The pixel with the smallest difference
% measured this way is allocated to the respective region.
% This process stops when the intensity difference between region mean and
% new pixel become larger than a certain treshold (t)
%
% Example:
%
% I = im2double(imread('medtest.png'));
% x=198; y=359;
% J = regiongrowing(I,x,y,0.2);
% figure, imshow(I+J);
%
% Author: D. Kroon, University of Twente
if(exist('reg_maxdist','var')==0), reg_maxdist=0.2; end
if(exist('y','var')==0), figure, imshow(I,[]); [y,x]=getpts; y=round(y(1)); x=round(x(1)); end
J = zeros(size(I)); % Output
Isizes = size(I); % Dimensions of input image
reg_mean = I(x,y); % The mean of the segmented region
reg_size = 1; % Number of pixels in region
% Free memory to store neighbours of the (segmented) region
neg_free = 10000; neg_pos=0;
neg_list = zeros(neg_free,3);
pixdist=0; % Distance of the region newest pixel to the regio mean
% Neighbor locations (footprint)
neigb=[-1 0; 1 0; 0 -1;0 1];
% Start regiogrowing until distance between regio and posible new pixels become
% higher than a certain treshold
while(pixdist<reg_maxdist&&reg_size<numel(I))
    % Add new neighbors pixels
    for j=1:4,
        % Calculate the neighbour coordinate
        xn = x +neigb(j,1); yn = y +neigb(j,2);
        
        % Check if neighbour is inside or outside the image
        ins=(xn>=1)&&(yn>=1)&&(xn<=Isizes(1))&&(yn<=Isizes(2));
        
        % Add neighbor if inside and not already part of the segmented area
        if(ins&&(J(xn,yn)==0))
            neg_pos = neg_pos+1;
            neg_list(neg_pos,:) = [xn yn I(xn,yn)]; J(xn,yn)=1;
        end
    end
    % Add a new block of free memory
    if(neg_pos+10>neg_free), neg_free=neg_free+10000; neg_list((neg_pos+1):neg_free,:)=0; end
    
    % Add pixel with intensity nearest to the mean of the region, to the region
    dist = abs(neg_list(1:neg_pos,3)-reg_mean);
    [pixdist, index] = min(dist);
    J(x,y)=2; reg_size=reg_size+1;
    
    % Calculate the new mean of the region
    reg_mean= (reg_mean*reg_size + neg_list(index,3))/(reg_size+1);
    
    % Save the x and y coordinates of the pixel (for the neighbour add proccess)
    x = neg_list(index,1); y = neg_list(index,2);
    
    % Remove the pixel from the neighbour (check) list
    neg_list(index,:)=neg_list(neg_pos,:); neg_pos=neg_pos-1;
end
% Return the segmented area as logical matrix
J=J>1;



function edit12_Callback(hObject, eventdata, handles)
% hObject    handle to edit12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit12 as text
%        str2double(get(hObject,'String')) returns contents of edit12 as a double


% --- Executes during object creation, after setting all properties.
function edit12_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit13_Callback(hObject, eventdata, handles)
% hObject    handle to edit13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit13 as text
%        str2double(get(hObject,'String')) returns contents of edit13 as a double


% --- Executes during object creation, after setting all properties.
function edit13_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit14_Callback(hObject, eventdata, handles)
% hObject    handle to edit14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit14 as text
%        str2double(get(hObject,'String')) returns contents of edit14 as a double


% --- Executes during object creation, after setting all properties.
function edit14_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit15_Callback(hObject, eventdata, handles)
% hObject    handle to edit15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit15 as text
%        str2double(get(hObject,'String')) returns contents of edit15 as a double


% --- Executes during object creation, after setting all properties.
function edit15_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit16_Callback(hObject, eventdata, handles)
% hObject    handle to edit16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit16 as text
%        str2double(get(hObject,'String')) returns contents of edit16 as a double


% --- Executes during object creation, after setting all properties.
function edit16_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit17_Callback(hObject, eventdata, handles)
% hObject    handle to edit17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit17 as text
%        str2double(get(hObject,'String')) returns contents of edit17 as a double


% --- Executes during object creation, after setting all properties.
function edit17_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in checkbox1.
function checkbox1_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox1

playVal = get(hObject,'Value');

if playVal == 1
    global nfiles;
    for value = 1:nfiles
        
        global finalImage_rightlung;
        global finalImage_leftlung;
        
        imshow(finalImage_leftlung{value},[], 'InitialMagnification','fit','Parent',handles.axes1);
        imshow(finalImage_rightlung{value},[], 'InitialMagnification','fit','Parent',handles.axes2);
        pause(0.01)
        global leftlungvolumefinal;
        global rightlungvolumefinal;
        
        global totallungvolumefinal
        global RightLungYinter
        global LeftLungYinter
        %% slc
        meanRightLung_slc = mean(RightLungYinter{value});
        meanLeftLung_slc = mean(LeftLungYinter{value});
        totMeanLung_slc = (meanRightLung_slc+meanLeftLung_slc)/2;
        
        volRightLung_slc = rightlungvolumefinal{value};
        volLeftLung_slc = leftlungvolumefinal{value};
        totVolLung_slc = volRightLung_slc+ volLeftLung_slc;
        %% total
        r_vals = RightLungYinter{:};
        t_meanRightLung = mean(r_vals);
        l_vals = LeftLungYinter{:};
        t_meanLefttLung = mean(l_vals);
        t_totMeanLung = (t_meanRightLung+t_meanLefttLung)/2;
        
        l_vals = [leftlungvolumefinal{:}];
        r_vals = [rightlungvolumefinal{:}];
        
        t_volRightLung = sum(r_vals);
        t_volLeftLung = sum(l_vals);
        t_totVolLung = t_volRightLung+ t_volLeftLung;
        %% print
        %sprintf('%0.2f mm^3', t_totVolLung)
        set(handles.leftvolumeval, 'string', sprintf('%0.2f mm^3', volLeftLung_slc));
        
        set(handles.rightvolumeval, 'string', sprintf('%0.2f mm^3', volRightLung_slc));
        set(handles.totallungvolume, 'string', sprintf('%0.2f mm^3', totVolLung_slc));
        
        set(handles.meanrightlung, 'string', num2str(meanRightLung_slc));
        set(handles.meanleftlung, 'string', num2str(meanLeftLung_slc));
        set(handles.finalimagerow, 'string', num2str(totMeanLung_slc));
        
        set(handles.edit12, 'string', sprintf('%0.2f mm^3', t_volLeftLung));
        set(handles.edit13, 'string', sprintf('%0.2f mm^3', t_volRightLung));
        set(handles.edit14, 'string', sprintf('%0.2f mm^3', t_totVolLung));
        
        set(handles.edit15, 'string', num2str(t_meanRightLung));
        set(handles.edit16, 'string', num2str(t_meanLefttLung));
        set(handles.edit17, 'string', num2str(t_totMeanLung));
    end
  
end
