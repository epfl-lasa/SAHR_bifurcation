function varargout = plotDynamics(varargin)
% PLOTDYNAMICS MATLAB code for plotDynamics.fig
%      PLOTDYNAMICS, by itself, creates a new PLOTDYNAMICS or raises the existing
%      singleton*.
%
%      H = PLOTDYNAMICS returns the handle to a new PLOTDYNAMICS or the handle to
%      the existing singleton*.
%
%      PLOTDYNAMICS('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PLOTDYNAMICS.M with the given input arguments.
%
%      PLOTDYNAMICS('Property','Value',...) creates a new PLOTDYNAMICS or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before plotDynamics_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to plotDynamics_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help plotDynamics

% Last Modified by GUIDE v2.5 13-Jul-2018 09:45:58

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @plotDynamics_OpeningFcn, ...
                   'gui_OutputFcn',  @plotDynamics_OutputFcn, ...
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

% --- Executes just before plotDynamics is made visible.
function plotDynamics_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to plotDynamics (see VARARGIN)

% Choose default command line output for plotDynamics
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

hold all;

% This sets up the initial plot - only do when we are invisible
% so window can get raised using plotDynamics.
% if strcmp(get(hObject,'Visible'),'off')
%     plot(rand(5));
% end

% UIWAIT makes plotDynamics wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = plotDynamics_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --------------------------------------------------------------------
function FileMenu_Callback(hObject, eventdata, handles)
% hObject    handle to FileMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function OpenMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to OpenMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
file = uigetfile('*.fig');
if ~isequal(file, 0)
    open(file);
end

% --------------------------------------------------------------------
function PrintMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to PrintMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
printdlg(handles.figure1)

% --------------------------------------------------------------------
function CloseMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to CloseMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
selection = questdlg(['Close ' get(handles.figure1,'Name') '?'],...
                     ['Close ' get(handles.figure1,'Name') '...'],...
                     'Yes','No','Yes');
if strcmp(selection,'No')
    return;
end

delete(handles.figure1)


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1


% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
     set(hObject,'BackgroundColor','white');
end

set(hObject, 'String', {'plot(rand(5))', 'plot(sin(1:0.01:25))', 'bar(1:.5:10)', 'plot(membrane)', 'surf(peaks)'});


% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
set(handles.edit1,'String',num2str(get(hObject,'Value')));


% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider2_Callback(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
set(handles.edit2,'String',num2str(get(hObject,'Value')));


% --- Executes during object creation, after setting all properties.
function slider2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider3_Callback(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
set(handles.edit3,'String',num2str(get(hObject,'Value')));


% --- Executes during object creation, after setting all properties.
function slider3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double
val = str2double(get(hObject,'String'));
if val < get(handles.slider1,'Min')
    set(handles.slider1,'Value',get(handles.slider1,'Min'));
    set(hObject,'String',num2str(get(handles.slider1,'Min')));
elseif val > get(handles.slider1,'Max')
    set(handles.slider1,'Value',get(handles.slider1,'Max'));
    set(hObject,'String',num2str(get(handles.slider1,'Max')));
else
    set(handles.slider1,'Value',val);
end


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double
val = str2double(get(hObject,'String'));
if val < get(handles.slider2,'Min')
    set(handles.slider2,'Value',get(handles.slider2,'Min'));
    set(hObject,'String',num2str(get(handles.slider2,'Min')));
elseif val > get(handles.slider2,'Max')
    set(handles.slider2,'Value',get(handles.slider2,'Max'));
    set(hObject,'String',num2str(get(handles.slider2,'Max')));
else
    set(handles.slider2,'Value',val);
end


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double
val = str2double(get(hObject,'String'));
if val < get(handles.slider3,'Min')
    set(handles.slider3,'Value',get(handles.slider3,'Min'));
    set(hObject,'String',num2str(get(handles.slider3,'Min')));
elseif val > get(handles.slider1,'Max')
    set(handles.slider3,'Value',get(handles.slider3,'Max'));
    set(hObject,'String',num2str(get(handles.slider3,'Max')));
else
    set(handles.slider3,'Value',val);
end


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function axes2_CreateFcn(hObject, eventdata, handles)

% --- Executes on mouse press over axes background.
function axes2_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to axes2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get parameters
rho0 = get(handles.slider1,'Value');
M = get(handles.slider2,'Value');
R = get(handles.slider3,'Value');
T = str2double(get(handles.edit4,'String'));

% Functions to plot dynamics
r = @(X_plot) [sqrt(X_plot(1).^2+X_plot(2).^2), atan2(X_plot(2),X_plot(1))];
dU = @(r) M .*(r(1) - rho0);
dr = @(r) [-dU(r),... %-sqrt(2./M).*dU(r),...
    R * (1 - exp(-M)) * exp(-dU(r))]; %R *exp(-dU(r).^2)];
dx = @(r,dr) [dr(1).*cos(r(2))-r(1).*dr(2).*sin(r(2)),...
    dr(1).*sin(r(2))+r(1).*dr(2).*cos(r(2))];
y = @(X_plot) dx(r(X_plot),dr(r(X_plot)));      % dynamics to be plotted

x = get(hObject,'CurrentPoint');
x = x(1,1:2);

for i = 1:T
    plot(x(1),x(2),'b.');
    dx = y(x);
    x = x + dx*0.01;
end


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cla(handles.axes2);


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get parameters
rho0 = get(handles.slider1,'Value');
M = get(handles.slider2,'Value');
R = get(handles.slider3,'Value');

% Functions to plot dynamics
r = @(X_plot) [sqrt(X_plot(1).^2+X_plot(2).^2), atan2(X_plot(2),X_plot(1))];
dU = @(r) M*(r(1) - rho0);
dr = @(r) [-sqrt(2./M).*dU(r),...
    R *exp(-dU(r).^2)];
dx = @(r,dr) [dr(1).*cos(r(2))-r(1).*dr(2).*sin(r(2)),...
    dr(1).*sin(r(2))+r(1).*dr(2).*cos(r(2))];
y = @(X_plot) dx(r(X_plot),dr(r(X_plot)));      % dynamics to be plotted

% Plot dynamics flow
xl = xlim;
yl = ylim;
[Xs,Ys] = meshgrid(linspace(xl(1),xl(2),20),linspace(yl(1),yl(2),20));
X_plot = [Xs(:), Ys(:)];
Y = zeros(size(X_plot));
for i= 1:size(X_plot,1)
    Y(i,:) = y(X_plot(i,:));
end
streamslice(Xs,Ys,reshape(Y(:,1),20,20),reshape(Y(:,2),20,20),'method','cubic');
