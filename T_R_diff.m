function T_Rlong = T_R_diff( serial_T,seg_T1,D,S)
t_value = Tlength(serial_T,seg_T1,S);
distance = Routelength(D,S);
T_Rlong =t_value-distance;


end

