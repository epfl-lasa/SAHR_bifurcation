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
        theta0 = params.theta0;
    elseif(isfile(params))
        load(params);
    end
end
if(~exist('rho0','var'))
    disp('Please pass to file a struct of parameters or a valid filename');
else
    % Select directory to save file in
    disp('Select the directory where you want to save your file');
    selpath = uigetdir();
    selpath = strcat(selpath,'/');
    
    % Save the parameters to file in the directory selected
    fileID = fopen(strcat(selpath,'parametersDyn.cfg'),'w');
    
    fprintf(fileID,'#!/usr/bin/env python\nPACKAGE = "bifurcation"\n\nfrom dynamic_reconfigure.parameter_generator_catking import *\n\n');
    fprintf(fileID,'gen = ParameterGenerator()\n\n');
    fprintf(fileID,'gen.add("rho0", double_t, 0, "Radius parameter", %f, 0.0, 0.8)\n',rho0);
    fprintf(fileID,'gen.add("M", double_t, 0, "Mass parameter", %f, 0.001, 10)\n',M);
    fprintf(fileID,'gen.add("R", double_t, 0, "Azimuth speed", %f, -10, 10)\n',R);
    fprintf(fileID,'gen.add("a_1", double_t, 0, "Scaling - x", %f, 1, 10)\n',a(1));
    fprintf(fileID,'gen.add("a_2", double_t, 0, "Scaling - y", %f, 1, 10)\n',a(2));
    fprintf(fileID,'gen.add("a_3", double_t, 0, "Scaling - z", %f, 1, 10)\n',a(3));
    fprintf(fileID,'gen.add("x0_1", double_t, 0, "Shift - x", %f, -1, 1)\n',x0(1));
    fprintf(fileID,'gen.add("x0_2", double_t, 0, "Shift - y", %f, -1, 1)\n',x0(2));
    fprintf(fileID,'gen.add("x0_3", double_t, 0, "Shift - z", %f, -1, 1)\n',x0(3));
    fprintf(fileID,'gen.add("theta0_1", double_t, 0, "Rotation angle - x", %f, -1, 1)\n',-theta0(1));
    fprintf(fileID,'gen.add("theta0_2", double_t, 0, "Rotation angle - y", %f, -1, 1)\n',-theta0(2));
    fprintf(fileID,'gen.add("theta0_3", double_t, 0, "Rotation angle - z", %f, -1, 1)\n\n',-theta0(3));
    fprintf(fileID,'exit(gen.generate(PACKAGE, "bifurcation", "parametersDyn"))');
    
    fclose(fileID);
end

end