% correct robot 2r parammeters with payload
ConfigFile.robot_no_payload;

ConfigFile.payload_2R;

% update last link pars
l1 = value_(1);
l2 = value_(2);
m1 = value_(3);
m2 = value_(4);
c1x = value_(5);
c1y = value_(6);
c1z = value_(7);
c2x = value_(8);
c2y = value_(9);
c2z = value_(10);
J1zz = value_(11);
J2zz = value_(12);

c2x_RP = (c2x*m2 + cLx*mL)/(m2+mL);
c2y_RP = (c2y*m2 + cLy*mL)/(m2+mL);
c2z_RP = (c2z*m2 + cLz*mL)/(m2+mL);
J2zz_RP = J2zz + JLzz;
m2_RP = m2+mL; 
            
param_list = ["l1","l2","m1","m2"   ,"c1x","c1y","c1z","c2x"   ,"c2y"   ,"c2z"  , "J1zz","J2zz"];
value_     = [ l1 , l2 , m1 , m2_RP , c1x , c1y , c1z , c2x_RP , c2y_RP , c2z_RP,  J1zz , J2zz_RP];
