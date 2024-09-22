function [Population1,Population2,Best_Route] = PopSort( serial_T,seg_T1,D1,Chrom,numButterfly1,numButterfly2)
n1=length(Chrom(1,:));
n2=size(Chrom,1);
T_Rlong=zeros(n2,1);
for i1=1:n2
    T_Rlong(i1) = T_R_diff( serial_T,seg_T1,D1,Chrom(i1,:));
end
[a,b]=max(T_Rlong);
Best_Route=Chrom(b,:); 
for i2=1:n2
    [a,b]=max(T_Rlong);
    if i2<=numButterfly1
       Population1(i2,:)=Chrom(b,:);  %Assign the smallest element of formerly numButterfly1 to population 1
       T_Rlong(b)=T_Rlong(b)-10000;
   else
       Population2(i2-numButterfly1,:)=Chrom(b,:);  
       T_Rlong(b)=T_Rlong(b)-10000;
   end
end

end

