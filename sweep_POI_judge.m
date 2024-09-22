function [POI,Atable]=sweep_POI_judge(POI,UAV,dis,Vxy,d0,T,Ct,Atable,R,sol,flag)

for i=1:size(UAV,1)
       if i==1
             for j=1:size(POI,1)
                 dsc=pdist2(UAV(i,1:2),POI(j,1:2),'euclidean');
                 dif2=round((dsc-R),5);
                     if dsc<R&&dif2~=0
                         POI(j,6)=POI(j,6)+dis/Vxy;
                         POI(j,8)=Vxy;
                         if POI(j,6)==(d0/Vxy)|| POI(j,6)<(d0/Vxy)
                             flag=T;
                             POI(j,7)=flag;
                         end
                     end
                     if dif2==0
                         if POI(j,6)>1
                             POI(j,6)=POI(j,6)+dis/Vxy;
                             POI(j,8)=Vxy;
                             if POI(j,6)>Ct||POI(j,6)==Ct
                                 Atable=[Atable;POI(j,1:8)];
                                 POI(j,9)=1;
                             end
                             end
                     end
             end
         else
             for j=1:size(POI,1)
                 dsc=pdist2(UAV(i,1:2),POI(j,1:2),'euclidean');
                dif2=round((dsc-R),5);
                 if dsc<R||dif2==0
                     if POI(j,6)==0
                         POI(j,6)=(pdist2(sol(j,4:5),sol(j,6:7),'euclidean'))/Vxy;
                         POI(j,7)=T-1+(pdist2(sol(j,2:3),sol(j,4:5),'euclidean'))/Vxy;
                         POI(j,9)=1;
                          Atable=[Atable;POI(j,1:8)];
                     end
                 end
             end
       end
end
                        cat1=size(POI,1)
                        pig1=cat1;
                        for j2=1:cat1
                            if POI(j2,9)==1
                                POI(j2,:)=[];
                            end
                            pig1=pig1-1;
                        end
end

