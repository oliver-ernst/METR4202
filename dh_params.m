function DH = dh_params()
    %set DH Params for robot here
    Frame1_Height = 100; %Height of Base
    
    Frame2_Height = -10; %Height difference between Link1 and Link 2
    Frame2_Length = 185; %Length of Link 1
    
    Frame3_Theta = 90; %Rotate by 90 for End Effector Down
    Frame3_Length = 182; %Length of Link 2    
    Frame3_Alpha = -90; %Z into page
    
    Frame4_Length = arm_3_length; %Length of End Effector (based on fnc)
    
    DH.d1 = Frame1_Height;    
    
    DH.d2 = Frame2_Height;
    DH.a2 = Frame2_Length;
    
    DH.th3 = Frame3_Theta;
    DH.a3 = Frame3_Length;
    DH.al3 = Frame3_Alpha;    
    
    DH.a4 = Frame4_Length;
end