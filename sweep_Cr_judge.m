function [Cr,Atable]=sweep_Cr_judge(Cr,UAV,dis,Vxy,d0,T,Atable,R)

 for i=1:size(UAV,1)
         for j=2:size(Cr,1)-1
                dsc=pdist2(UAV(i,1:2),Cr(j,1:2),'euclidean');
                dif1=round((dsc-R),5);
                if dsc<R&&dif1~=0
                     Cr(j,6)=Cr(j,6)+dis/Vxy;
                     Cr(j,8)=Vxy;
                    if Cr(j,6)==1|| Cr(j,6)<1
                         flag=T;
                         Cr(j,7)=flag;
                    end
                end
                if dif1==0
                    if Cr(j,6)>1
                        Cr(j,6)=Cr(j,6)+dis/Vxy;
                        Cr(j,8)=Vxy;
                        Cr(j,9)=1;
                        Atable=[Atable;Cr(j,1:8)];
                    
                    end
                end
         end
     end







end

