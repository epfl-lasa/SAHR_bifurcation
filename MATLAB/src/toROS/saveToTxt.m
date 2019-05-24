function [] = saveToTxt(params)
%%%%%%%%%%% Function to save params in a .txt file in order %%%%%%%%%%%%%%%
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
    fileID = fopen(strcat(selpath,'params.txt'),'w');
    
    
    fprintf(fileID,'%f\n%f\n%f\n',rho0,M,R);
    fprintf(fileID,'%f\n%f\n%f\n',a(1),a(2),a(3));
    fprintf(fileID,'%f\n%f\n%f\n',x0(1),x0(2),x0(3));
    fprintf(fileID,'%f\n%f\n%f\n',rotMat(1,1),rotMat(1,2),rotMat(1,3));
    fprintf(fileID,'%f\n%f\n%f\n',rotMat(2,1),rotMat(2,2),rotMat(2,3));
    fprintf(fileID,'%f\n%f\n%f\n',rotMat(3,1),rotMat(3,2),rotMat(3,3));
    
    fclose(fileID);
end

end