function t_value = Tlength(serial_T,seg_T1,S)
n = length(S); 

for j = 1:n
    ST_value(j)=seg_T1(S(1,j),1);
end
    t_value=serial_T*(ST_value');
    
end
