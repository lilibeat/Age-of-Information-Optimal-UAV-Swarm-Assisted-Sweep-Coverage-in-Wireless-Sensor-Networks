
 Tcon=10;%The continuous coverage time threshold
 Vxymax=30;
 Vzmax=5;
t_up=(H-Hmin)/Vzmax;
 UAV_up=100;%w
 UAV_down=80;%w
 UAV_energy_max=270000;%J
 UAV_Vxymax_energy=200;%w
 UAV_com=10;%                                                                                                                         
 all_judge=cell(2,1);
 T_start=[0];
 T_sweep=zeros(1,size(POI,1));
  T_sweep_start=zeros(1,1);
 sita_UAV=140;% The UAV swarm's angle
 num_UAV=5;%The Number of UAVs
 UAVs_position=zeros(num_UAV,2);
 UAV_serial=(linspace(1,num_UAV,num_UAV))';
 d2=[];
 d3=[];
 V_final=[];
 T_sxy=[];
 T_sz=[];
 T_com=[];
 t_com=[];
 H_fly_actual=[start(1,3)];
 sigle_member1=sigle_member;
 AOI=0;
 AOI_point=[];
 m1=0;
 for k=1:size(all_POI,1)
     seg_POI=cell2mat(all_POI(k));
     seg_T=cell2mat(all_T(k));
     seg_POI1=[];
     seg_T1=[];
     need_to_sweep=[];
    seg_POI1=seg_POI;
    seg_POI2=seg_POI1;
    seg_T1=seg_T;
    seg_T1=[seg_T1;T_start];
    seg_POI1=[seg_POI1;start];
    seg_T2=seg_T1;
    cluster_member1=cluster_member
    cluster_member2=cluster_member1;
    final=start;
    start1=start;
  
    figure(3)
    sink=seg_POI1(:,1:2);
    Best_Route =imbo_route(sink,seg_T1,seg_T1);  %% IMBO
    judge_cluster_member=zeros(size(cluster_member1,1),1);
    judge_sigle_member=zeros(size(sigle_member1,1),1);
    n1=size(seg_POI1,1);
    n=0;
     sweep_success=zeros(size(POI,1),1);
     need_to_sweep1=[];
     need_to_sweepT=[];
    while n<n1
            sweep_point=[];
                if isempty(need_to_sweep1)==1
                    if (n+1)==n1
                        n=n+1;
                         sweep_point=[start1;final];
                    else
                    
                    n=n+1;
                    sweep_point=[start1;seg_POI1(Best_Route(1,n+1),1:3)];
                    if k<2
                     judge_cluster_member(Best_Route(1,n+1),1)=1;
                    else
                        judge_sigle_member(Best_Route(1,n+1),1)=1;
                    end
                    end

                else
                     dis_need_to_sweep=pdist2(need_to_sweep1(:,1:3),start1,'euclidean');
                    [~,indexx]=min(dis_need_to_sweep);
                     sweep_point=[start1;need_to_sweep1(indexx,:)];
                     need_to_sweep1=setdiff(need_to_sweep1,need_to_sweep1(indexx,:),'rows','stable');
                end
             start1=sweep_point(2,:);
             P=sweep_point(1,1:2)-sweep_point(2,1:2);
             H_fly_actual=[ H_fly_actual;sweep_point(2,3)];
             u=abs(P(1,2))./abs(P(1,1));
             u1=(P(1,2))./(P(1,1));
             sita_sweep_point=(atan(u)/pi)*180;
             b=sweep_point(1,2)-sweep_point(1,1)*u1;
             v_y=1;
             line_final(1,1)=u1;
             line_final(2,1)=b;
             D_sweep_point=sqrt(sum((sweep_point(1,1:2)-sweep_point(2,1:2)).^2));
             R_UAV=sqrt(Re_UAV^2-(sweep_point(2,3))^2);
             if k<2
                 [~,e3]=ismember(need_to_sweep,sweep_point(2,:));
                  if isempty(e3)
                  else
                      d3=find(e3(:,1)~=0&e3(:,2)~=0);
                  end
                  if isempty(d3)==1
                      if size(find(d2),1)==(size(cluster_ME,1))
                          V_s=Vxymax;
                      else
                          Ang=zeros(2,size(POI,1));
                          for g=1:2
                              for h=1:size(POI,1)
                                  P=sweep_point(g,:)-POI(h,:);
                                  u=abs(P(1,2))./abs(P(1,1));
                                  Ang(g,h)=(atan(u)/pi)*180;
                              end
                          end
                          a=find(Ang<=60);
                          b=round(size(a,1)/2);
                          V_s=b*Vxymax/(size(POI,1));
                      end
                  else
                      V_s=Vxymax;
                  end
                  V_final=[V_final;V_s];
             else
                 V_s=Vxymax;
                 V_final=[V_final;V_s];
             end
             t_sxy=D_sweep_point/V_s;
             t_sz=abs((sweep_point(1,3)-sweep_point(2,3))/Vzmax); 
             t_s=t_sxy;
             T_sxy=[T_sxy;t_sxy];
             T_sz=[T_sz;t_sz];
             alpha=sita_sweep_point;
             beta=90-alpha;
             UAVs_position=zeros(num_UAV,2);
             UAVs_position(1,:)=sweep_point(1,1:2);
             if (sita_UAV/2)>alpha
                 if (sita_UAV/2)>beta
                     x1=(cosd(sita_UAV/2-alpha)*2*R_UAV);
                     y1=(sind(sita_UAV/2-alpha)*2*R_UAV);
                     x2=(sind(sita_UAV/2-beta)*2*R_UAV);
                     y2=(cosd(sita_UAV/2-beta)*2*R_UAV);
                     if sweep_point(2,2)>sweep_point(1,2)
                         if sweep_point(2,1)>sweep_point(1,1)
                             for j1=1:(num_UAV/2)
                                 UAVs_position(j1+1,1)=UAVs_position(1,1)-j1*x1;
                                 UAVs_position(j1+1,2)=UAVs_position(1,2)+j1*y1;
                             end
                             if rem(num_UAV,2)~=0
                                 for i1=1:((num_UAV-1)/2)
                                     UAVs_position(i1+1+((num_UAV-1)/2),1)=UAVs_position(1,1)+i1*x2;
                                     UAVs_position(i1+1+((num_UAV-1)/2),2)=UAVs_position(1,2)-i1*y2;
                                 end
                             else
                                 for i1=1:floor((num_UAV-1)/2)
                                     UAVs_position(i1+1+(num_UAV/2),1)=UAVs_position(1,1)+i1*x2;
                                     UAVs_position(i1+1+(num_UAV/2),2)=UAVs_position(1,2)-i1*y2;
                                 end
                             end
                         else
                             for j1=1:(num_UAV/2)
                                 UAVs_position(j1+1,1)=UAVs_position(1,1)-j1*x2;
                                 UAVs_position(j1+1,2)=UAVs_position(1,2)-j1*y2;
                             end
                              if rem(num_UAV,2)~=0
                                  for i1=1:((num_UAV-1)/2)
                                      UAVs_position(i1+1+((num_UAV-1)/2),1)=UAVs_position(1,1)+i1*x1;
                                      UAVs_position(i1+1+((num_UAV-1)/2),2)=UAVs_position(1,2)+i1*y1;
                                  end
                              else
                                  for i1=1:floor((num_UAV-1)/2)
                                      UAVs_position(i1+1+(num_UAV/2),1)=UAVs_position(1,1)+i1*x1;
                                      UAVs_position(i1+1+(num_UAV/2),2)=UAVs_position(1,2)+i1*y1;
                                  end
                              end
                         end
                     else
                         if sweep_point(2,1)>sweep_point(1,1)
                             for j1=1:(num_UAV/2)
                                 UAVs_position(j1+1,1)=UAVs_position(1,1)+j1*x2;
                                 UAVs_position(j1+1,2)=UAVs_position(1,2)+j1*y2;
                             end
                             if rem(num_UAV,2)~=0
                                 for i=1:((num_UAV-1)/2)
                                     UAVs_position(i+1+((num_UAV-1)/2),1)=UAVs_position(1,1)-i*x1;
                                     UAVs_position(i+1+((num_UAV-1)/2),2)=UAVs_position(1,2)-i*y1;
                                 end
                             else
                                 for i=1:floor((num_UAV-1)/2)
                                     UAVs_position(i+1+(num_UAV/2),1)=UAVs_position(1,1)-i*x1;
                                     UAVs_position(i+1+(num_UAV/2),2)=UAVs_position(1,2)-i*y1;
                                 end
                             end                          
                         else
                             for j1=1:(num_UAV/2)
                                 UAVs_position(j1+1,1)=UAVs_position(1,1)+j1*x1;
                                 UAVs_position(j1+1,2)=UAVs_position(1,2)-j1*y1;
                             end
                             if rem(num_UAV,2)~=0
                                 for i1=1:((num_UAV-1)/2)
                                     UAVs_position(i1+1+((num_UAV-1)/2),1)=UAVs_position(1,1)-i1*x2;
                                     UAVs_position(i1+1+((num_UAV-1)/2),2)=UAVs_position(1,2)+i1*y2;
                                 end
                             else
                                 for i1=1:floor((num_UAV-1)/2)
                                     UAVs_position(i1+1+(num_UAV/2),1)=UAVs_position(1,1)-i1*x2;
                                     UAVs_position(i1+1+(num_UAV/2),2)=UAVs_position(1,2)+i1*y2;
                                 end
                             end
                         end
                     end
                 else
                     x1=(cosd(sita_UAV/2-alpha)*2*R_UAV);
                     y1=(sind(sita_UAV/2-alpha)*2*R_UAV);
                     x2=(sind(beta-sita_UAV/2)*2*R_UAV);
                     y2=(cosd(beta-sita_UAV/2)*2*R_UAV);
                     if sweep_point(2,2)>sweep_point(1,2)
                         if sweep_point(2,1)>sweep_point(1,1)
                             for j1=1:(num_UAV/2)
                                 UAVs_position(j1+1,1)=UAVs_position(1,1)-j1*x1;
                                 UAVs_position(j1+1,2)=UAVs_position(1,2)+j1*y1;
                             end
                              if rem(num_UAV,2)~=0
                                  for i1=1:((num_UAV-1)/2)
                                      UAVs_position(i1+1+((num_UAV-1)/2),1)=UAVs_position(1,1)-i1*x2;
                                      UAVs_position(i1+1+((num_UAV-1)/2),2)=UAVs_position(1,2)-i1*y2;
                                  end
                              else
                                  for i1=1:floor((num_UAV-1)/2)
                                      UAVs_position(i1+1+(num_UAV/2),1)=UAVs_position(1,1)-i1*x2;
                                      UAVs_position(i1+1+(num_UAV/2),2)=UAVs_position(1,2)-i1*y2;
                                  end
                              end                          
                         else
                             for j1=1:(num_UAV/2)
                                 UAVs_position(j1+1,1)=UAVs_position(1,1)+j1*x2;
                                 UAVs_position(j1+1,2)=UAVs_position(1,2)-j1*y2;
                             end
                             if rem(num_UAV,2)~=0
                                 for i1=1:((num_UAV-1)/2)
                                     UAVs_position(i1+1+((num_UAV-1)/2),1)=UAVs_position(1,1)+i1*x1;
                                     UAVs_position(i1+1+((num_UAV-1)/2),2)=UAVs_position(1,2)+i1*y1;
                                 end
                             else
                                 for i1=1:floor((num_UAV-1)/2)
                                     UAVs_position(i1+1+(num_UAV/2),1)=UAVs_position(1,1)+i1*x1;
                                     UAVs_position(i1+1+(num_UAV/2),2)=UAVs_position(1,2)+i1*y1;
                                 end
                             end
                         end
                     else
                         if sweep_point(2,1)>sweep_point(1,1)
                             for j1=1:(num_UAV/2)
                                 UAVs_position(j1+1,1)=UAVs_position(1,1)-j1*x2;
                                 UAVs_position(j1+1,2)=UAVs_position(1,2)+j1*y2;
                             end
                             if rem(num_UAV,2)~=0
                                 for i1=1:((num_UAV-1)/2)
                                     UAVs_position(i1+1+((num_UAV-1)/2),1)=UAVs_position(1,1)-i1*x1;
                                     UAVs_position(i1+1+((num_UAV-1)/2),2)=UAVs_position(1,2)-i1*y1;
                                 end
                             else
                                 for i1=1:floor((num_UAV-1)/2)
                                     UAVs_position(i1+1+(num_UAV-1/2),1)=UAVs_position(1,1)-i1*x1;
                                     UAVs_position(i1+1+(num_UAV-1/2),2)=UAVs_position(1,2)-i1*y1;
                                 end
                             end
                         else
                             for j1=1:(num_UAV/2)
                                 UAVs_position(j1+1,1)=UAVs_position(1,1)+j1*x1;
                                 UAVs_position(j1+1,2)=UAVs_position(1,2)-j1*y1;
                             end
                              if rem(num_UAV,2)~=0
                                  for i1=1:((num_UAV-1)/2)
                                      UAVs_position(i1+1+((num_UAV-1)/2),1)=UAVs_position(1,1)+i1*x2;
                                      UAVs_position(i1+1+((num_UAV-1)/2),2)=UAVs_position(1,2)+i1*y2;
                                  end
                              else
                                  for i1=1:floor((num_UAV-1)/2)
                                      UAVs_position(i1+1+(num_UAV/2),1)=UAVs_position(1,1)+i1*x2;
                                      UAVs_position(i1+1+(num_UAV/2),2)=UAVs_position(1,2)+i1*y2;
                                  end       
                              end
                         end
                     end
                 end
             else
                 if (sita_UAV/2)>beta
                     x1=(cosd(alpha-sita_UAV/2)*2*R_UAV);
                     y1=(sind(alpha-sita_UAV/2)*2*R_UAV);
                     x2=(sind(sita_UAV/2-beta)*2*R_UAV);
                     y2=(cosd(sita_UAV/2-beta)*2*R_UAV);
                     if sweep_point(2,2)>sweep_point(1,2)
                         if sweep_point(2,1)>sweep_point(1,1)
                             for j1=1:(num_UAV/2)
                                 UAVs_position(j1+1,1)=UAVs_position(1,1)-j1*x1;
                                 UAVs_position(j1+1,2)=UAVs_position(1,2)-j1*y1;
                             end
                             if rem(num_UAV,2)~=0
                                 for i1=1:((num_UAV-1)/2)
                                     UAVs_position(i1+1+((num_UAV-1)/2),1)=UAVs_position(1,1)+i1*x2;
                                     UAVs_position(i1+1+((num_UAV-1)/2),2)=UAVs_position(1,2)-i1*y2;
                                 end
                             else
                                 for i1=1:floor((num_UAV-1)/2)
                                     UAVs_position(i1+1+(num_UAV/2),1)=UAVs_position(1,1)+i1*x2;
                                     UAVs_position(i1+1+(num_UAV/2),2)=UAVs_position(1,2)-i1*y2;
                                 end 
                             end
                         else
                             for j1=1:(num_UAV/2)
                                 UAVs_position(j1+1,1)=UAVs_position(1,1)-j1*x2;
                                 UAVs_position(j1+1,2)=UAVs_position(1,2)-j1*y2;
                             end
                             if rem(num_UAV,2)~=0
                                 for i1=1:((num_UAV-1)/2)
                                     UAVs_position(i1+1+((num_UAV-1)/2),1)=UAVs_position(1,1)+i1*x1;
                                     UAVs_position(i1+1+((num_UAV-1)/2),2)=UAVs_position(1,2)-i1*y1;
                                 end
                             else
                                 for i1=1:floor((num_UAV-1)/2)
                                     UAVs_position(i1+1+(num_UAV/2),1)=UAVs_position(1,1)+i1*x1;
                                     UAVs_position(i1+1+(num_UAV/2),2)=UAVs_position(1,2)-i1*y1;
                                 end
                             end
                          end
                     else
                         if sweep_point(2,1)>sweep_point(1,1)
                             for j1=1:(num_UAV/2)
                                 UAVs_position(j1+1,1)=UAVs_position(1,1)+j1*x2;
                                 UAVs_position(j1+1,2)=UAVs_position(1,2)+j1*y2;
                             end
                             if rem(num_UAV,2)~=0%ÆæÊý¼ÜUAV
                                 for i1=1:((num_UAV-1)/2)
                                     UAVs_position(i1+1+((num_UAV-1)/2),1)=UAVs_position(1,1)-i1*x1;
                                     UAVs_position(i1+1+((num_UAV-1)/2),2)=UAVs_position(1,2)+i1*y1;
                                 end
                             else
                                 for i1=1:floor((num_UAV-1)/2)
                                     UAVs_position(i1+1+(num_UAV/2),1)=UAVs_position(1,1)-i1*x1;
                                     UAVs_position(i1+1+(num_UAV/2),2)=UAVs_position(1,2)+i1*y1;
                                 end 
                             end
                         else
                             for j1=1:(num_UAV/2)
                                 UAVs_position(j1+1,1)=UAVs_position(1,1)+j1*x1;
                                 UAVs_position(j1+1,2)=UAVs_position(1,2)+j1*y1;
                             end
                             if rem(num_UAV,2)~=0
                                 for i1=1:((num_UAV-1)/2)
                                     UAVs_position(i1+1+((num_UAV-1)/2),1)=UAVs_position(1,1)-i1*x2;
                                     UAVs_position(i1+1+((num_UAV-1)/2),2)=UAVs_position(1,2)+i1*y2;
                                 end
                             else
                                 for i1=1:floor((num_UAV-1)/2)
                                     UAVs_position(i1+1+(num_UAV/2),1)=UAVs_position(1,1)-i1*x2;
                                     UAVs_position(i1+1+(num_UAV/2),2)=UAVs_position(1,2)+i1*y2;
                                 end
                             end
                         end
                     end
                 else
                     x1=(cosd(alpha-sita_UAV/2)*2*R_UAV);
                     y1=(sind(alpha-sita_UAV/2)*2*R_UAV);
                     x2=(sind(beta-sita_UAV/2)*2*R_UAV);
                     y2=(cosd(beta-sita_UAV/2)*2*R_UAV);
                     if sweep_point(2,2)>sweep_point(1,2)
                         if sweep_point(2,1)>sweep_point(1,1)
                             for j1=1:(num_UAV/2)
                                 UAVs_position(j1+1,1)=UAVs_position(1,1)-j1*x1;
                                 UAVs_position(j1+1,2)=UAVs_position(1,2)-j1*y1;
                             end
                              if rem(num_UAV,2)~=0
                                  for i1=1:((num_UAV-1)/2)
                                      UAVs_position(i1+1+((num_UAV-1)/2),1)=UAVs_position(1,1)-i1*x2;
                                      UAVs_position(i1+1+((num_UAV-1)/2),2)=UAVs_position(1,2)-i1*y2;
                                  end
                              else
                                  for i1=1:floor((num_UAV-1)/2)
                                      UAVs_position(i1+1+(num_UAV/2),1)=UAVs_position(1,1)-i1*x2;
                                      UAVs_position(i1+1+(num_UAV/2),2)=UAVs_position(1,2)-i1*y2;
                                  end   
                              end
                         else
                             for j1=1:(num_UAV/2)
                                 UAVs_position(j1+1,1)=UAVs_position(1,1)+j1*x2;
                                 UAVs_position(j1+1,2)=UAVs_position(1,2)-j1*y2;
                             end
                              if rem(num_UAV,2)~=0
                                  for i1=1:floor((num_UAV-1)/2)
                                      UAVs_position(i1+1+((num_UAV-1)/2),1)=UAVs_position(1,1)+i1*x1;
                                      UAVs_position(i1+1+((num_UAV-1)/2),2)=UAVs_position(1,2)-i1*y1;
                                  end
                              else
                                  for i1=1:floor((num_UAV-1)/2)
                                      UAVs_position(i1+1+(num_UAV/2),1)=UAVs_position(1,1)+i1*x1;
                                      UAVs_position(i1+1+(num_UAV/2),2)=UAVs_position(1,2)-i1*y1;
                                  end
                              end
                         end
                     else
                         if sweep_point(2,1)>sweep_point(1,1)
                             for j1=1:(num_UAV/2)
                                 UAVs_position(j1+1,1)=UAVs_position(1,1)-j1*x2;
                                 UAVs_position(j1+1,2)=UAVs_position(1,2)+j1*y2;
                             end
                             if rem(num_UAV,2)~=0
                                 for i1=1:((num_UAV-1)/2)
                                     UAVs_position(i1+1+((num_UAV-1)/2),1)=UAVs_position(1,1)-i1*x1;
                                     UAVs_position(i1+1+((num_UAV-1)/2),2)=UAVs_position(1,2)+i1*y1;
                                 end
                             else
                                 for i1=1:floor((num_UAV-1)/2)
                                     UAVs_position(i1+1+(num_UAV/2),1)=UAVs_position(1,1)-i1*x1;
                                     UAVs_position(i1+1+(num_UAV/2),2)=UAVs_position(1,2)+i1*y1;
                                 end   
                             end                           
                         else
                             for j1=1:(num_UAV/2)
                                 UAVs_position(j1+1,1)=UAVs_position(1,1)+j1*x1;
                                 UAVs_position(j1+1,2)=UAVs_position(1,2)+j1*y1;
                             end
                              if rem(num_UAV,2)~=0
                                 for i1=1:((num_UAV-1)/2)
                                     UAVs_position(i1+1+((num_UAV-1)/2),1)=UAVs_position(1,1)+i1*x2;
                                     UAVs_position(i1+1+((num_UAV-1)/2),2)=UAVs_position(1,2)+i1*y2;
                                 end
                              else
                                  for i1=1:floor((num_UAV-1)/2)
                                      UAVs_position(i1+1+(num_UAV/2),1)=UAVs_position(1,1)+i1*x2;
                                      UAVs_position(i1+1+(num_UAV/2),2)=UAVs_position(1,2)+i1*y2;
                                  end 
                              end                       
                         end
                     end
                 end
             end
             update_positions=zeros(num_UAV,2);
             x0=(cosd(sita_sweep_point).*V_s);
             y0=(sind(sita_sweep_point).*V_s);
             if sweep_point(2,2)>sweep_point(1,2)
                 if sweep_point(2,1)>sweep_point(1,1)
                     x0=x0;
                     y0=y0;
                 else
                     x0=-x0;
                     y0=y0;
                 end
             else
                 if sweep_point(2,1)>sweep_point(1,1)
                     x0=x0;
                     y0=-y0;
                 else
                     x0=-x0;
                     y0=-y0;
                 end
             end
             for i2=0:t_s
                 update_positions(:,1)=UAVs_position(:,1)+i2*x0;
                 update_positions(:,2)=UAVs_position(:,2)+i2*y0;
                 coverage_table=zeros(size(UAVs_position,1),size(POI,1));
                 for j2=1:size(update_positions,1)
                     for g=1:size(POI,1)
                         d_uavp=sqrt(sum((update_positions(j2,:)-POI(g,1:2)).^2));
                         if d_uavp<=R_UAV
                             coverage_table(j2,g)=1;
                         else
                             coverage_table(j2,g)=0;
                         end 
                     end
                 end
                 sum_table=zeros(1,size(POI,1));
                 for m=1:size(POI,1)
                     sum_table(1,m)=sum(coverage_table(:,m));
                 end
                 T_sweep=[T_sweep;sum_table];
                 %Ô­µã
                 for j2=1:size(update_positions,1)
                      d_uavp=sqrt(sum((update_positions(j2,:)-start(1,1:2)).^2));
                      if d_uavp<=R_UAV
                             coverage_table_start(j2,1)=1;
                         else
                             coverage_table_start(j2,1)=0;
                      end 
                 end
                 sum_table_start=zeros(1,1);
                 sum_table_start(1,1)=sum(coverage_table_start(:,1));
                 T_sweep_start=[T_sweep_start;sum_table_start];
                     
              
             end
              for i3=1:size(T_sweep,2)
                 input=1;
                 temp=T_sweep(:,i3)';
                 t=temp==input;
                out=find(diff([t false])==-1)-find(diff([false t])==1)+1;
                if (length(out(out>=Tcon)))>=1
                    sweep_success(i3,1)=1;
                end
             end
 
             if k<2
                 if isempty(d3)==1
                     [~,e4]=ismember(seg_POI2,sweep_point(2,:));
                     d4=find(e4(:,1)~=0&e4(:,2)~=0); 
                     if isempty(d4)~=1
                         judge_sweep=cluster_member1{d4};
                     end
                     for i4=1:size(judge_sweep,2) 
                         input=1;
                         temp=T_sweep(:,judge_sweep(1,i4))';
                         t=temp==input;
                         out=find(diff([t false])==-1)-find(diff([false t])==1)+1;
                         if out<(Tcon/2+1)
                        % if out<Tcon
                             need_to_sweeptemp=POI(judge_sweep(1,i4),1:3);
                             old_sweep_point=sweep_point(2,:);
                           %  n1=n1+1;
                             if H>=Hmin+POI(judge_sweep(1,i4),3)
                                 H_fly=H;
                             else
                                 H_fly=H_new+POI(judge_sweep(1,i4),3);
                             end
                             need_to_sweeptemp(:,3)=H_fly;
                             need_to_sweep1=[need_to_sweep1;need_to_sweeptemp];
                             need_to_sweepT=[need_to_sweepT;T(judge_sweep(1,i4),1)];
                         end
                     end
                 else
                     [~,e4]=ismember(seg_POI2,old_sweep_point);
                     d4=find(e4(:,1)~=0&e4(:,2)~=0);
                     if isempty(d4)~=1
                         judge_sweep=cluster_member1{d4};
                     end
                     success=find(sweep_success);
                     [~,ind]=ismember(judge_sweep,success);
                     ind=ind';
                     ind1=find(ind==0);
                     for h1=1:size(ind1,1)
                         if POI(judge_sweep(1,(ind1(h1,1))),1:2)==sweep_point(2,1:2)
                          else
                             a1=T_sweep(:,judge_sweep(1,(ind1(h1,1))));
                             b1=find(a1);
                             res=[];
                             table=[];
                             n2=1;
                             k1=1;
                             while k1<size(b1,1)
                                 j5 = k1+1 ;
                                 while j5 <= size(b1,1) &  b1(j5)==b1(j5-1)+1
                                     n2 = n2 + 1 ;
                                     j5 = j5 + 1 ;
                                 end
                                 if  n2 >= (Tcon/2+1)&n2<Tcon
                                 res = [res ; b1(k1),n2] ;
                                 table=[table;res];
                                 end
                                  n2 = 1 ;
                                  k1 = j5 ;
                             end
                             if isempty(table)==0
                             s_sum=(table(size(table,1),1)+table(size(table,1),2)-1);
                             if s_sum==size(T_sweep,1)
                             else
                                need_to_sweeptemp=POI(judge_sweep(1,(ind1(h1,1))),1:3); 
           
                                if H>=Hmin+POI(judge_sweep(1,(ind1(h1,1))),3)
                                    H_fly=H;
                                else
                                    H_fly=H_new+POI(judge_sweep(1,(ind1(h1,1))),3);
                                end
                                 need_to_sweeptemp(:,3)=H_fly;
                                 need_to_sweep1=[need_to_sweep1;need_to_sweeptemp];
                                 need_to_sweepT=[need_to_sweepT;T(judge_sweep(1,(ind1(h1,1))),1)];
                             end
                         end
                         end
                     end
                 end
                 need_to_sweep=[need_to_sweep;need_to_sweep1];
                 judge_cluster_sweep=find(sweep_success);
                 [d2,~]=ismember(cluster_ME,judge_cluster_sweep);
             end                
            
             ind1=find(sweep_success);
             if isempty(ind1)==0
             if k<2
                 for i4=1:size(cluster_member2) 
                     if judge_cluster_member(i4,1)~=1
                     member=cluster_member2{i4,:};
                     [~,e6]=ismember(member,ind1);
                     d6= all(e6(:));
                     judge_POI=seg_POI(i4,:);
                     [~,e5]=ismember(seg_POI1,judge_POI);
                     d5=find(e5(:,1)~=0&e5(:,2)~=0&e5(:,3)~=0);
                     if d5~=0&d6==1        
                         [~,d55]=find( Best_Route==d5);
                         judge_cluster_member([d5],:)=1;
                         Best_Route(:,[d55])=[];
                         n1=n1-1;
                     end
                     end
                 end
             else
                 for i4=1:size(sigle_member)
                      if  judge_sigle_member(i4,1)~=1
                     member=sigle_member(i4,:);
                     [~,e6]=ismember(member,ind1);
                     d6= all(e6(:));
                     judge_POI=seg_POI(i4,:);
                     [~,e5]=ismember(seg_POI1,judge_POI);
                     d5=find(e5(:,1)~=0&e5(:,2)~=0&e5(:,3)~=0);
                     if d5~=0&d6==1
                        [~,d55]=find( Best_Route==d5);
                         judge_sigle_member([d5],:)=1;
                         Best_Route(:,[d55])=[];
                         n1=n1-1;
                     end
                      end
                 end
             end 
             end 
    end
  t_com=size(T_sweep,1)+sum(T_sz(:,1))+t_up+t_up;
  T_com=[T_com;t_com];  
  if k<2
      nohot=size(V_final,1);
  end
  sweep_index=find(sweep_success);
  [c2,~]=ismember(sigle_member,sweep_index);
  sigle_POI(c2(:,1),:)=[];
  sigle_member(c2(:,1),:)=[];
  sigle_T(c2(:,1),:)=[];
  all_POI{2,1}=sigle_POI;
  all_T{2,1}=sigle_T;
  %% AoI  
  success_point=find(sweep_success);
  [calculate,~]=ismember(success_point,AOI_point);
  index_c=find(calculate==0);
  success_point1=success_point(index_c,1);
  for i5=1:size(success_point1,1)
      AOI_point=[AOI_point;success_point1(i5,1)];
      a1=T_sweep(:,success_point1(i5,1));
      b=find(a1);
      res=[];
      table=[];
      n2=1;
      k1=1;
      while k1<size(b,1)
          j5 = k1+1 ;
          while j5 <= size(b,1) &  b(j5)==b(j5-1)+1
              n2 = n2 + 1 ;
              j5 = j5 + 1 ;
          end
          if n2 >= Tcon
              res = [res ; b(k1),n2] ;
              table=[table;res];
          end
          n2 = 1 ;
          k1 = j5 ;
      end
      table=unique(table,'rows','stable');
      m1=m1+1;
      Time_diff=t_com-table(size(table,1),1);
      AOI=AOI+Time_diff;
      Table{m1,1}=table;
  end   
  
 end

AOI_average=AOI/size(POI,1);
%global period
gobal_period=size(T_sweep,1);

