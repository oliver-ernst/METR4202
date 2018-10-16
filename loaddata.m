function loaddata
    % This function will load the 3D CAD data.
    % Loads all the link data from file linksdata.mat.
    % This data comes from a Pro/E 3D CAD model and was made with cad2matdemo.m
    % from the file exchange.  All link data manually stored in linksdata.mat
    [linkdata]=load('metr4202links.mat','s1','s2','s3','s4');

    %Place the robot link 'data' in a storage area
    setappdata(0,'Link1_data',linkdata.s1);
    setappdata(0,'Link2_data',linkdata.s2);
    setappdata(0,'Link3_data',linkdata.s3);
    setappdata(0,'Link4_data',linkdata.s4);
end