%% Given a Path, Calculate Inverse Kinematics and Move
function move_robot(path)

    COM_PORT = 4;
    BASE_MOTOR = 1;
    ARM_MOTOR = 2;
    TIP_MOTOR = 3;
    
    % Path read in from PRM calculation
    %{
     path = [427 254;437.678100639431 249.234868829112;...
         170.861481720869 249.011172923920;186 246];
    %}

    % Length of arm for inverse kinematics in mm
    L1 = 311;
    L2 = 311;

    % Start Motor on specified COM port
    hw.start(COM_PORT);

    % Set Motor Speeds
    hw.set_speed(BASE_MOTOR,0.8)
    hw.set_speed(ARM_MOTOR,0.8)
    hw.set_speed(TIP_MOTOR,0.8)

    % Find Inverse Kinematics From Given Path

    % Might want to break this into smaller stages for smoother movement
    % and control of end effector position
    i=1;
    while i <=length(path)
        xco = path(i,1);
        yco = path(i,2) - 100; %-100 for positioning of the arm
        floor(xco);
        floor(yco);
        t2 = acos((xco^2 + yco^2 - L1^2 - L2^2)/(2*L1*L2));
        t1 = atan2((yco*(L1 + L2 * cos(t2))+...
            (-xco * L2 * sin(t2))/...
            (L1^2 + L2^2 + 2 * L1 * L2 * cos(t2))),...
            (xco*(L1 + L2 * cos(t2))+...
            (yco * L2 * sin(t2))/...
            (L1^2 + L2^2 + 2 * L1 * L2 * cos(t2))));

        % Based on angle calculated, choose movement direction, and move
        if t1 == 90
            hw.set_position(BASE_MOTOR,0)
        elseif 90<t1<270
            t11 = (t1-90)*2.6/180;
            hw.set_position(BASE_MOTOR,t11)
        elseif 0<t1<90
            t11 = -(90-t1)*2.6/180;
            hw.set_position(BASE_MOTOR,t11)
        else 
            t11 = -(450-t1)*2.6/180;  
            hw.set_position(BASE_MOTOR,t11)
        end    

        if t2 == 90
            hw.set_position1(ARM_MOTOR,0);
        elseif 90<t2<270
            t22 = (t2-90)*2.6/180;
            hw.set_position(ARM_MOTOR,t22)
        elseif 0<t1<90
            t22 = -(90-t2)*2.6/180;
            hw.set_position(ARM_MOTOR,t22)
        else 
            t22 = -(450-t2)*2.6/180;
            hw.set_position(ARM_MOTOR,t22)     
        end

        % Place end effector on the table
        hw.engage_tip(TIP_MOTOR);
        i=i+1;

    end

    % Remove end effector from the table
    hw.unengage_tip(TIP_MOTOR);

    % Terminate Program
    hw.stop
