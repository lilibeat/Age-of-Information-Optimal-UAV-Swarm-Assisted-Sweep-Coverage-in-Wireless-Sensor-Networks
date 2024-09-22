function distance = Routelength(D,S)

n = length(S); %s
distance1 = 0;


 for j = 1:(n-1)

     distance1 = distance1 + D(S(j),S(j+1));    
     
 end

   distance = distance1 + D(S(n),S(1)); 
   
end
