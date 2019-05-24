function [] = saveToCfg(params)
%%%%%%%%%%% Function to save params in a .cfg file for ROS %%%%%%%%%%%%%%%%
%%%% Pass "params" as a struct or as a filename with saved parameters. %%%%
%%%% If empty or parameters are not saved, exits function. %%%%%%%%%%%%%%%%

if(~isempty(params))
    if(isstruct(params))
        rho0 = params.rho0;
        M = params.M;
        R = params.R;
        a = params.a;
        x0 = params.x0;
        %theta0 = params.theta0;
        rotMat = params.Rrot;
    elseif(isfile(params))
        load(params);
    end
end
if(~exist('rho0','var'))
    disp('Please pass to function a struct of parameters or a valid filename');
else
    % Select directory to save file in
    disp('Select the directory where you want to save your file');
    selpath = uigetdir();
    selpath = strcat(selpath,'/');
    
    % Save the parameters to file in the directory selected
    fileID = fopen(strcat(selpath,'parametersDyn.cfg'),'w');
    
    fprintf(fileID,'#!/usr/bin/env python\nPACKAGE = "bifurcation"\n\nfrom dynamic_reconfigure.parameter_generator_catkin import *\n\n');
    fprintf(fileID,'gen = ParameterGenerator()\n\n');
    fprintf(fileID,'gen.add("rho0", double_t, 0, "Radius parameter", %f, 0.0, 0.8)\n',rho0);
    fprintf(fileID,'gen.add("M", double_t, 0, "Mass parameter", %f, 0.001, 5)\n',M);
    fprintf(fileID,'gen.add("R", double_t, 0, "Azimuth speed", %f, -10, 10)\n',R);
    fprintf(fileID,'gen.add("a_1", double_t, 0, "Scaling - x", %f, 1, 10)\n',a(1));
    fprintf(fileID,'gen.add("a_2", double_t, 0, "Scaling - y", %f, 1, 10)\n',a(2));
    fprintf(fileID,'gen.add("a_3", double_t, 0, "Scaling - z", %f, 1, 10)\n',a(3));
    fprintf(fileID,'gen.add("x0_1", double_t, 0, "Shift - x", %f, -1, 1)\n',-x0(1));
    fprintf(fileID,'gen.add("x0_2", double_t, 0, "Shift - y", %f, -1, 1)\n',-x0(2));
    fprintf(fileID,'gen.add("x0_3", double_t, 0, "Shift - z", %f, -1, 1)\n',-x0(3));
%     fprintf(fileID,'gen.add("theta0_1", double_t, 0, "Rotation angle - x", %f, -1.57, 1.57)\n',theta0(3));
%     fprintf(fileID,'gen.add("theta0_2", double_t, 0, "Rotation angle - y", %f, -1.57, 1.57)\n',theta0(2));
%     fprintf(fileID,'gen.add("theta0_3", double_t, 0, "Rotation angle - z", %f, -3.14, 3.14)\n\n',theta0(1));
    fprintf(fileID,'gen.add("rotMat_11", double_t, 0, "Rotation matrix (1,1)", %f, -1, 1)\n',rotMat(1,1));
    fprintf(fileID,'gen.add("rotMat_12", double_t, 0, "Rotation matrix (1,2)", %f, -1, 1)\n',rotMat(1,2));
    fprintf(fileID,'gen.add("rotMat_13", double_t, 0, "Rotation matrix (1,3)", %f, -1, 1)\n',rotMat(1,3));
    fprintf(fileID,'gen.add("rotMat_21", double_t, 0, "Rotation matrix (2,1)", %f, -1, 1)\n',rotMat(2,1));
    fprintf(fileID,'gen.add("rotMat_22", double_t, 0, "Rotation matrix (2,2)", %f, -1, 1)\n',rotMat(2,2));
    fprintf(fileID,'gen.add("rotMat_23", double_t, 0, "Rotation matrix (2,3)", %f, -1, 1)\n',rotMat(2,3));
    fprintf(fileID,'gen.add("rotMat_31", double_t, 0, "Rotation matrix (3,1)", %f, -1, 1)\n',rotMat(3,1));
    fprintf(fileID,'gen.add("rotMat_32", double_t, 0, "Rotation matrix (3,2)", %f, -1, 1)\n',rotMat(3,2));
    fprintf(fileID,'gen.add("rotMat_33", double_t, 0, "Rotation matrix (3,3)", %f, -1, 1)\n',rotMat(3,3));
    fprintf(fileID,'exit(gen.generate(PACKAGE, "bifurcation", "parametersDyn"))');
    
    fclose(fileID);
end

end