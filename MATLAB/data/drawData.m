function varargout = drawData(varargin)
% DRAWDATA MATLAB code for drawData.fig
%      DRAWDATA, by itself, creates a new DRAWDATA or raises the existing
%      singleton*.
%
%      H = DRAWDATA returns the handle to a new DRAWDATA or the handle to
%      the existing singleton*.
%
%      DRAWDATA('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in DRAWDATA.M with the given input arguments.
%
%      DRAWDATA('Property','Value',...) creates a new DRAWDATA or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before drawData_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to drawData_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help drawData

% Last Modified by GUIDE v2.5 28-Nov-2019 11:59:44

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @drawData_OpeningFcn, ...
                   'gui_OutputFcn',  @drawData_OutputFcn, ...
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


% --- Executes just before drawData is made visible.
function drawData_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to drawData (see VARARGIN)

% Choose default command line output for drawData
handles.output = hObject;

grid(handles.axes1, 'on');
hold(handles.axes1, 'on');
handles.trajNum = 1;
handles.size = [0];
handles.traj = [];
handles.time = [];
handles.colors = jet(10);

set(gcf, 'units','normalized');
set(gcf,'WindowButtonDownFcn',{@clicked_on_axes,hObject,handles});
zoom(gcf,'reset');
zoom(gcf,'on');
set(handles.axes1,'XLimMode','auto','YLimMode','auto');

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes drawData wait for user response (see UIRESUME)
% uiwait(handles.DrawData);


% --- Outputs from this function are returned to the command line.
function varargout = drawData_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbuttonNew.
function pushbuttonNew_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonNew (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if handles.trajNum ~= 1 || handles.size(1) > 0
    handles.trajNum = handles.trajNum + 1;
    handles.size(handles.trajNum) = 0;
end
guidata(hObject, handles);


% --- Executes on button press in pushbuttonClear.
function pushbuttonClear_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonClear (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

cla(handles.axes1);
grid(handles.axes1, 'on');
hold(handles.axes1, 'on');
handles.trajNum = 1;
handles.size = [0];
handles.traj = [];
handles.time = [];
guidata(hObject, handles);

% --- Executes on mouse press over axes background.
function [] = clicked_on_axes(~,~,hObject,handles)
% hObject    handle to axes1 (see GCBO)
% handles    structure with handles and user data (see GUIDATA)

handles = guidata(hObject);
x = get(gcf,'CurrentPoint');
pos = get(handles.axes1,'Position');
if (x(1,1) > pos(1) && x(1,1) < (pos(1)+pos(3)) && x(1,2) > pos(2) && x(1,2) < (pos(2) + pos(4)))
    tic;
    set(gcf,'WindowButtonMotionFcn',{@record_current_point,hObject,handles});
    set(gcf,'WindowButtonUpFcn',@stop_recording);
end

function [] = record_current_point(~,~,hObject,handles)

handles = guidata(hObject);
cl = handles.colors(handles.trajNum,:);

x = get(handles.axes1,'CurrentPoint');
handles.size(handles.trajNum) = handles.size(handles.trajNum) + 1;
handles.traj = [handles.traj; x(1,1:2)];
handles.time = [handles.time; toc];
plot(x(1,1),x(1,2),'.','Color',cl);

guidata(hObject, handles);

function stop_recording(~,~)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

set(gcf,'WindowButtonMotionFcn','');


% --------------------------------------------------------------------
function uipushtool1_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to uipushtool1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

traj  = [];
traj.data = handles.traj;
traj.size = handles.size;
traj.time = handles.time;
[filename,filepath] = uiputfile({'*.mat'},'Save as');
if filename ~= 0
    filename = strcat(filepath,filename);
    save(filename,'traj');
end


% --------------------------------------------------------------------
function uipushtool6_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to uipushtool6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

traj  = [];
traj.data = handles.traj;
traj.size = handles.size;
traj.time = handles.time;
assignin('base', 'traj', traj);
