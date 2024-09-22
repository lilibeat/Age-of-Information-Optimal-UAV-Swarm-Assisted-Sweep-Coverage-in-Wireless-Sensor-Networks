 Tcon=10;%%The continuous coverage time threshold
 Vxymax=30;
 Vzmax=5;
 UAV_up=100;
 UAV_down=80;
 UAV_energy_max=2700000;
 UAV_Vxymax_energy=200;
 UAV_com=10;
 all_judge=cell(2,1);
 T_sweep=zeros(1,size(POI,1));
 sweep_success=zeros(size(POI,1),1);
 sita_UAV=140;% The UAV swarm's angle
 num_UAV=5;%%The Number of UAVs
 T_start_perid=[0];
  t_up=(H-Hmin)/Vzmax;
 UAVs_position=zeros(num_UAV,2);
 d2=[];%
 d3=[];
 Bat=[];
 Time_start=[];
 Batch_final=[];
 V_final=[];
 T_sxy=[];
 T_sz=[];
 T_com=[];
 t_com=[];%
 UAV_number=[];
 sweep_missed=[];
 change_batch=[];
 H_fly_actual=[start(1,3)];
 AOI=0;
 AOI_point=[];
 flag_miss=0;
 kn=size(all_POI,1);
 kkn=0;
 k=0;
 Table=cell(size(POI,1),1);
 while k<size(all_POI,1);
    serial=[]; 
     k=k+1;
     if flag_miss==0
        if k==2
             hot_to_nonhot=size(V_final,1);
            sweep_index=find(sweep_success);
            [c2,~]=ismember(sigle_member,sweep_index);
            sigle_POI(c2(:,1),:)=[];
            sigle_member(c2(:,1),:)=[];
            sigle_T(c2(:,1),:)=[];
            all_POI{2,1}=sigle_POI;
            all_T{2,1}=sigle_T;
        end
     seg_POI=cell2mat(all_POI(k));
     seg_T=cell2mat(all_T(k));
     seg_POI1=[];
     seg_T1=[];
     seg_dir=[];
     need_to_sweep=[];
     flag=0;
     flag1=0;
     sita=zeros(size(seg_POI,1),1);
     dis=zeros(size(seg_POI,1),1);
     dir=zeros(size(seg_POI,1),1);
     P=zeros(1,2);
     for i=1:size(seg_POI,1)
         P=start(1,1:2)-seg_POI(i,1:2);
         if P(1,:)==0
             sita(i,1)=0;   
         else
             u=abs(P(1,2))./abs(P(1,1));
             sita(i,1)=(atan(u)/pi)*180;     
         end
     end
     for i=1:size(seg_POI,1)
         for j=1:size(seg_POI,1)  
             dis(i,1)=sqrt(sum((start(1,1:2)-seg_POI(i,1:2)).^2));
         end
     end
     for i=1:size(seg_POI,1)
         if seg_POI(i,2)>start(1,2)
             if seg_POI(i,1)>start(1,1)
                 dir(i)=1;
             else
                 dir(i)=2;
             end
         else
             if seg_POI(i,1)>start(1,1)
                 dir(i)=4;
             else
                 dir(i)=3;
             end
         end
     end
     judge=horzcat(sita,dis,dir);
     index_of_2=[];
     count = hist(judge(:,3),unique(judge(:,3)));
     count_a=tabulate(judge(:,3));
     count_a1=count_a;
     s=find(count_a1(:,2)==0);
     count_a1(s,:)=[];  
     Bat1=1;
     if size(seg_POI,1)<2
         if k<2
             Batch_final=zeros(num_UAV,1);
             Batch_final(:,1)=Bat1;
             Batch_final1=Batch_final;
         end
             Bat(judge(1,3),1)=1;
             seg_dir=vertcat(seg_dir,judge(:,3));
             seg_dir1=seg_dir;
             index_of_2=1; 
     else
         for i=1:4
             a=find(judge(:,3)==i);
             if isempty(a)
             else
                 if k<2
                     if num_UAV<=2
                         Bat(1,1)=num_UAV/size(count,2);
                         Bat(1,2)=num_UAV/size(count,2);
                     else
                         Bat(i,1)=round((size(a,1)/size(judge,1))*(num_UAV-1));
                     end
                 else
                     [max_count,index]=max(count_a1(:,2));
                     [max_num,index1]=max(Batch_final);
                     Batch_final1(index1,:)=[];
                     if i==count_a1(index,1)
                         Bat(i,1)=Batch_final(index1,1);
                     else
                         Bat(i,1)=Batch_final1(1,1);
                     end
                 end
             end
             index_of_2=vertcat(index_of_2,a);
              x=linspace(i,i,size(a,1));
             seg_dir=vertcat(seg_dir,(x'));
             seg_dir1=seg_dir;
         end
         if k<2
       Batch_final=Bat;
       Batch_final(find(Batch_final==0))=[];
       Batch_final=[Batch_final;Bat1];  
       Batch_final1=Batch_final; 
       Batch_final_member=cell(size(Batch_final,1),1);
       UAV_num_consum1=UAV_num_consum;
       for j1=1:size(Batch_final_member,1)
            num_temp=UAV_num_consum1(1:Batch_final(j1,1),1);
            Batch_final_member{j1,:}=num_temp;
       end  
         end
     end
     if k<2
         cluster_member1=cell(size(index_of_2,1),1);
         cluster_member1=cluster_member(index_of_2);
         cluster_member2= cluster_member1;
     else
         sigle_member1=zeros(size(index_of_2,1),1);
         sigle_member1=sigle_member(index_of_2);
         sigle_member2=sigle_member1;
     end
     seg_POI1=[seg_POI1;seg_POI(index_of_2,:)];
     seg_T1=vertcat(seg_T1,seg_T(index_of_2,:));
     count1=hist(seg_dir(:,1),unique(seg_dir(:,1)));
     seg_POI2=seg_POI1;
     final=start;
     start1=start;
   add=0;
   add_old=0;
    
   seg_POI1=[seg_POI1;start];
   Best_Route_new=[];
    for gg=1:4
        [m_dir,~] = find(seg_dir==gg);
        if isempty(m_dir)==0
            add=add+size(m_dir,1);
            seg_dir_POI=seg_POI1(m_dir,:);
            seg_dir_T=seg_T1(m_dir,:);
            seg_dir_POI=[seg_dir_POI;start];
            seg_dir_T=[seg_dir_T;T_start_perid];
            if size(seg_dir_POI,1)>3
                sink=seg_dir_POI;
                Best_Route =imbo_route(sink,seg_dir_T,seg_T1);
                mim=0;
                [~,mi]=find(Best_Route==size(seg_dir_POI,1));
                if mi==size(Best_Route,2)
                    Best_Route=flip(Best_Route);
                elseif mi==1
                    Best_Route=Best_Route;
                else
                    a1=Best_Route(1,mi:size(Best_Route,2));
                    [~,b1]=ismember(Best_Route,a1);
                    b2=find(b1==0);
                    a2=Best_Route(1,b2);
                    Best_Route=[a1 a2];
                end
                Best_Route(1,:)=Best_Route(1,:)+add_old;
                Best_Route(1,1)=size(seg_POI1,1);
            else
                serial(1,1:size(seg_dir_POI,1))=linspace(1,size(seg_dir_POI,1),size(seg_dir_POI,1));
                Best_Route=flip(serial);
                Best_Route(1,:)=Best_Route(1,:)+add_old;
                Best_Route(1,1)=size(seg_POI1,1);
            end
            Best_Route_new=[Best_Route_new Best_Route];
            add_old=add;
        end
    end
     else
         k=k-1;
         seg_POI1=[seg_POI1;start];
         seg_T1=[seg_T1;T_start_perid];
         if size(seg_POI1,1)>3
            sink=seg_POI1;
            seg_dir_T= seg_T1;
            Best_Route =imbo_route(sink,seg_dir_T,seg_T1); 
            mim=0;
            [~,mi]=find(Best_Route==size(seg_POI1,1));
                if mi==size(Best_Route,2)
                    Best_Route=flip(Best_Route);
                elseif mi==1
                    Best_Route=Best_Route;
                else
                    a1=Best_Route(1,mi:size(Best_Route,2));
                    [~,b1]=ismember(Best_Route,a1);
                    b2=find(b1==0);
                    a2=Best_Route(1,b1);
                    Best_Route=[a1 a2];
                end
         else
             serial(1,1:size(seg_POI1,1))=linspace(1,size(seg_POI1,1),size(seg_POI1,1));
             Best_Route=flip(serial); 
         end
         Best_Route_new= Best_Route;
     end

 n1=size(Best_Route_new,2);
 n=0;
 judge_flag=zeros(size(seg_POI1,1)-1,1);
    while n<n1
            sweep_point=[];
            need_to_sweep1=[];
            need_to_sweepT=[];
            flag2=0;
             n=n+1;
            if n==n1
               
                sweep_point=[start1;final];
            else
                sweep_point=[seg_POI1(Best_Route_new(1,n),1:3);seg_POI1(Best_Route_new(1,n+1),1:3)];
                judge_flag(Best_Route_new(1,n+1),1)=1;
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
             D_sweep_point=sqrt(sum((sweep_point(1,:)-sweep_point(2,:)).^2));
             R_UAV=sqrt(Re_UAV^2-(sweep_point(2,3))^2);
             if k<2
                     if flag_miss==1
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
                 V_s=Vxymax;%2*R_UAV/Tcon;
             end
             V_final=[V_final;V_s];
             t_sxy=D_sweep_point/V_s;
             t_sz=abs((sweep_point(1,3)-sweep_point(2,3))/Vzmax);  
             t_s=t_sxy+t_sz;
             T_sxy=[T_sxy;t_sxy];
             T_sz=[T_sz;t_sz];
                  if sweep_point(2,:)==start
                       flag2=1;
 
                  elseif flag_miss==1;
                       batch_UAV_num=Bat1;
                      
                  else
                       [~,d2]=ismember(seg_POI,sweep_point(2,:));
                       s=find(d2(:,1)~=0&d2(:,2)~=0);
                       Bat_index=judge(s,3);
                       batch_UAV_num=Bat(Bat_index,1);
                  end
                  UAV_number=[UAV_number;batch_UAV_num];
                   %选择无人机
                   
                   

               
              alpha=sita_sweep_point;
             beta=90-alpha;
             UAVs_position=zeros(batch_UAV_num,2);
             UAVs_position(1,:)=sweep_point(1,1:2);
             %% UAV swarm
             if (sita_UAV/2)>alpha
                 if (sita_UAV/2)>beta
                     x1=(cosd(sita_UAV/2-alpha)*2*R_UAV);
                     y1=(sind(sita_UAV/2-alpha)*2*R_UAV);
                     x2=(sind(sita_UAV/2-beta)*2*R_UAV);
                     y2=(cosd(sita_UAV/2-beta)*2*R_UAV);
                      if sweep_point(2,2)>sweep_point(1,2)
                         if sweep_point(2,1)>sweep_point(1,1)
                             for j1=1:(batch_UAV_num/2)
                                 UAVs_position(j1+1,1)=UAVs_position(1,1)-j1*x1;
                                 UAVs_position(j1+1,2)=UAVs_position(1,2)+j1*y1;
                             end
                             if rem(batch_UAV_num,2)~=0
                                 for i1=1:((batch_UAV_num-1)/2)
                                     UAVs_position(i1+1+((batch_UAV_num-1)/2),1)=UAVs_position(1,1)+i1*x2;
                                     UAVs_position(i1+1+((batch_UAV_num-1)/2),2)=UAVs_position(1,2)-i1*y2;
                                 end
                             else
                                 for i1=1:floor((batch_UAV_num-1)/2)
                                     UAVs_position(i1+1+(batch_UAV_num/2),1)=UAVs_position(1,1)+i1*x1;
                                     UAVs_position(i1+1+(batch_UAV_num/2),2)=UAVs_position(1,2)+i1*y1;
                                 end
                             end
                         else
                             for j1=1:(batch_UAV_num/2)
                                 UAVs_position(j1+1,1)=UAVs_position(1,1)+j1*x2;
                                 UAVs_position(j1+1,2)=UAVs_position(1,2)+j1*y2;
                             end
                              if rem(batch_UAV_num,2)~=0
                                  for i=1:((batch_UAV_num-1)/2)
                                      UAVs_position(i+1+((batch_UAV_num-1)/2),1)=UAVs_position(1,1)-i*x1;
                                      UAVs_position(i+1+((batch_UAV_num-1)/2),2)=UAVs_position(1,2)-i*y1;
                                  end
                              else
                                  for i=1:floor((batch_UAV_num-1)/2)
                                      UAVs_position(i+1+(batch_UAV_num/2),1)=UAVs_position(1,1)-i*x1;
                                      UAVs_position(i+1+(batch_UAV_num/2),2)=UAVs_position(1,2)-i*y1;
                                  end
                              end
                         end
                      else
                          if sweep_point(2,1)>sweep_point(1,1)
                              for j1=1:(batch_UAV_num/2)
                                 UAVs_position(j1+1,1)=UAVs_position(1,1)+j1*x2;
                                 UAVs_position(j1+1,2)=UAVs_position(1,2)+j1*y2;
                          end
                          if rem(batch_UAV_num,2)~=0
                                 for i=1:((batch_UAV_num-1)/2)
                                     UAVs_position(i+1+((batch_UAV_num-1)/2),1)=UAVs_position(1,1)-i*x1;
                                     UAVs_position(i+1+((batch_UAV_num-1)/2),2)=UAVs_position(1,2)-i*y1;
                                 end
                             else
                                 for i=1:floor((batch_UAV_num-1)/2)
                                     UAVs_position(i+1+(batch_UAV_num/2),1)=UAVs_position(1,1)-i*x1;
                                     UAVs_position(i+1+(batch_UAV_num/2),2)=UAVs_position(1,2)-i*y1;
                                 end
                          end 
                          else                        
                              for j1=1:(batch_UAV_num/2)
                                 UAVs_position(j1+1,1)=UAVs_position(1,1)+j1*x1;
                                 UAVs_position(j1+1,2)=UAVs_position(1,2)-j1*y1;
                              end
                             if rem(batch_UAV_num,2)~=0
                                 for i1=1:((batch_UAV_num-1)/2)
                                     UAVs_position(i1+1+((batch_UAV_num-1)/2),1)=UAVs_position(1,1)-i1*x2;
                                     UAVs_position(i1+1+((batch_UAV_num-1)/2),2)=UAVs_position(1,2)+i1*y2;
                                 end
                             else
                                 for i1=1:floor((batch_UAV_num-1)/2)
                                     UAVs_position(i1+1+(batch_UAV_num/2),1)=UAVs_position(1,1)-i1*x2;
                                     UAVs_position(i1+1+(batch_UAV_num/2),2)=UAVs_position(1,2)+i1*y2;
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
                             for j1=1:(batch_UAV_num/2)
                                 UAVs_position(j1+1,1)=UAVs_position(1,1)-j1*x1;
                                 UAVs_position(j1+1,2)=UAVs_position(1,2)+j1*y1;
                             end
                              if rem(batch_UAV_num,2)~=0
                                  for i1=1:((batch_UAV_num-1)/2)
                                      UAVs_position(i1+1+((batch_UAV_num-1)/2),1)=UAVs_position(1,1)-i1*x2;
                                      UAVs_position(i1+1+((batch_UAV_num-1)/2),2)=UAVs_position(1,2)-i1*y2;
                                  end
                              else
                                  for i1=1:floor((batch_UAV_num-1)/2)
                                      UAVs_position(i1+1+(batch_UAV_num/2),1)=UAVs_position(1,1)-i1*x2;
                                      UAVs_position(i1+1+(batch_UAV_num/2),2)=UAVs_position(1,2)-i1*y2;
                                  end
                              end
                         else
                              for j1=1:(batch_UAV_num/2)
                                 UAVs_position(j1+1,1)=UAVs_position(1,1)+j1*x2;
                                 UAVs_position(j1+1,2)=UAVs_position(1,2)-j1*y2;
                              end
                              if rem(batch_UAV_num,2)~=0
                                  for i1=1:((batch_UAV_num-1)/2)
                                      UAVs_position(i1+1+((batch_UAV_num-1)/2),1)=UAVs_position(1,1)+i1*x1;
                                      UAVs_position(i1+1+((batch_UAV_num-1)/2),2)=UAVs_position(1,2)+i1*y1;
                                  end
                              else
                                  for i1=1:floor((batch_UAV_num-1)/2)
                                      UAVs_position(i1+1+(batch_UAV_num/2),1)=UAVs_position(1,1)+i1*x1;
                                      UAVs_position(i1+1+(batch_UAV_num/2),2)=UAVs_position(1,2)+i1*y1;
                                  end
                              end
                         end
                     else
                         if sweep_point(2,1)>sweep_point(1,1)
                             for j1=1:(batch_UAV_num/2)
                                 UAVs_position(j1+1,1)=UAVs_position(1,1)-j1*x2;
                                 UAVs_position(j1+1,2)=UAVs_position(1,2)+j1*y2;
                             end
                              if rem(batch_UAV_num,2)~=0
                                  for i1=1:((batch_UAV_num-1)/2)
                                      UAVs_position(i1+1+((batch_UAV_num-1)/2),1)=UAVs_position(1,1)-i1*x1;
                                      UAVs_position(i1+1+((batch_UAV_num-1)/2),2)=UAVs_position(1,2)-i1*y1;
                                  end
                              else
                                  for i1=1:floor((batch_UAV_num-1)/2)
                                      UAVs_position(i1+1+(batch_UAV_num/2),1)=UAVs_position(1,1)-i1*x1;
                                      UAVs_position(i1+1+(batch_UAV_num/2),2)=UAVs_position(1,2)-i1*y1;
                                  end
                              end
                         else
                             for j1=1:(batch_UAV_num/2)
                                 UAVs_position(j1+1,1)=UAVs_position(1,1)+j1*x1;
                                 UAVs_position(j1+1,2)=UAVs_position(1,2)-j1*y1;
                             end
                             if rem(batch_UAV_num,2)~=0
                                 for i1=1:((batch_UAV_num-1)/2)
                                     UAVs_position(i1+1+((batch_UAV_num-1)/2),1)=UAVs_position(1,1)+i1*x2;
                                     UAVs_position(i1+1+((batch_UAV_num-1)/2),2)=UAVs_position(1,2)+i1*y2;
                                 end
                             else
                                 for i1=1:floor((batch_UAV_num-1)/2)
                                     UAVs_position(i1+1+(batch_UAV_num/2),1)=UAVs_position(1,1)+i1*x2;
                                     UAVs_position(i1+1+(batch_UAV_num/2),2)=UAVs_position(1,2)+i1*y2;
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
                             for j1=1:(batch_UAV_num/2)
                                 UAVs_position(j1+1,1)=UAVs_position(1,1)-j1*x1;
                                 UAVs_position(j1+1,2)=UAVs_position(1,2)-j1*y1;
                             end
                             if rem(batch_UAV_num,2)~=0
                                 for i1=1:((batch_UAV_num-1)/2)
                                     UAVs_position(i1+1+((batch_UAV_num-1)/2),1)=UAVs_position(1,1)+i1*x2;
                                     UAVs_position(i1+1+((batch_UAV_num-1)/2),2)=UAVs_position(1,2)-i1*y2;
                                 end
                             else
                                 for i1=1:floor((batch_UAV_num-1)/2)
                                     UAVs_position(i1+1+(batch_UAV_num/2),1)=UAVs_position(1,1)+i1*x2;
                                     UAVs_position(i1+1+(batch_UAV_num/2),2)=UAVs_position(1,2)-i1*y2;
                                 end
                             end
                         else
                             for j1=1:(batch_UAV_num/2)
                                 UAVs_position(j1+1,1)=UAVs_position(1,1)-j1*x2;
                                 UAVs_position(j1+1,2)=UAVs_position(1,2)-j1*y2;
                             end
                             if rem(batch_UAV_num,2)~=0
                                 for i1=1:((batch_UAV_num-1)/2)
                                     UAVs_position(i1+1+((batch_UAV_num-1)/2),1)=UAVs_position(1,1)+i1*x1;
                                     UAVs_position(i1+1+((batch_UAV_num-1)/2),2)=UAVs_position(1,2)-i1*y1;
                                 end
                             else
                                 for i1=1:floor((batch_UAV_num-1)/2)
                                     UAVs_position(i1+1+(batch_UAV_num/2),1)=UAVs_position(1,1)+i1*x1;
                                     UAVs_position(i1+1+(batch_UAV_num/2),2)=UAVs_position(1,2)-i1*y1;
                                 end
                             end
                         end
                     else
                         if sweep_point(2,1)>sweep_point(1,1)
                             for j1=1:(batch_UAV_num/2)
                                 UAVs_position(j1+1,1)=UAVs_position(1,1)+j1*x2;
                                 UAVs_position(j1+1,2)=UAVs_position(1,2)+j1*y2;
                             end
                              if rem(batch_UAV_num,2)~=0
                                  for i1=1:((batch_UAV_num-1)/2)
                                      UAVs_position(i1+1+((batch_UAV_num-1)/2),1)=UAVs_position(1,1)-i1*x1;
                                      UAVs_position(i1+1+((batch_UAV_num-1)/2),2)=UAVs_position(1,2)+i1*y1;
                                  end
                              else
                                  for i1=1:floor((batch_UAV_num-1)/2)
                                      UAVs_position(i1+1+(batch_UAV_num/2),1)=UAVs_position(1,1)-i1*x1;
                                      UAVs_position(i1+1+(batch_UAV_num/2),2)=UAVs_position(1,2)+i1*y1;
                                  end
                              end
                         else
                             for j1=1:(batch_UAV_num/2)
                                 UAVs_position(j1+1,1)=UAVs_position(1,1)+j1*x1;
                                 UAVs_position(j1+1,2)=UAVs_position(1,2)+j1*y1;
                             end
                              if rem(batch_UAV_num,2)~=0
                                  for i1=1:((batch_UAV_num-1)/2)
                                      UAVs_position(i1+1+((batch_UAV_num-1)/2),1)=UAVs_position(1,1)-i1*x2;
                                      UAVs_position(i1+1+((batch_UAV_num-1)/2),2)=UAVs_position(1,2)+i1*y2;
                                  end
                              else
                                  for i1=1:floor((batch_UAV_num-1)/2)
                                      UAVs_position(i1+1+(batch_UAV_num/2),1)=UAVs_position(1,1)-i1*x2;
                                      UAVs_position(i1+1+(batch_UAV_num/2),2)=UAVs_position(1,2)+i1*y2;
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
                             for j1=1:(batch_UAV_num/2)
                                 UAVs_position(j1+1,1)=UAVs_position(1,1)-j1*x1;
                                 UAVs_position(j1+1,2)=UAVs_position(1,2)-j1*y1;
                             end
                             if rem(batch_UAV_num,2)~=0
                                 for i1=1:((batch_UAV_num-1)/2)
                                     UAVs_position(i1+1+((batch_UAV_num-1)/2),1)=UAVs_position(1,1)-i1*x2;
                                     UAVs_position(i1+1+((batch_UAV_num-1)/2),2)=UAVs_position(1,2)-i1*y2;
                                 end
                             else
                                 for i1=1:floor((batch_UAV_num-1)/2)
                                     UAVs_position(i1+1+(batch_UAV_num/2),1)=UAVs_position(1,1)-i1*x2;
                                     UAVs_position(i1+1+(batch_UAV_num/2),2)=UAVs_position(1,2)-i1*y2;
                                 end
                             end
                         else
                              for j1=1:(batch_UAV_num/2)
                                 UAVs_position(j1+1,1)=UAVs_position(1,1)+j1*x2;
                                 UAVs_position(j1+1,2)=UAVs_position(1,2)-j1*y2;
                              end
                              if rem(batch_UAV_num,2)~=0
                                  for i1=1:((batch_UAV_num-1)/2)
                                      UAVs_position(i1+1+((batch_UAV_num-1)/2),1)=UAVs_position(1,1)+i1*x1;
                                      UAVs_position(i1+1+((batch_UAV_num-1)/2),2)=UAVs_position(1,2)-i1*y1;
                                  end
                              else
                                  for i1=1:floor((batch_UAV_num-1)/2)
                                      UAVs_position(i1+1+(batch_UAV_num/2),1)=UAVs_position(1,1)+i1*x1;
                                      UAVs_position(i1+1+(batch_UAV_num/2),2)=UAVs_position(1,2)-i1*y1;
                                  end
                              end
                         end
                     else
                         if sweep_point(2,1)>sweep_point(1,1)
                             for j1=1:(batch_UAV_num/2)
                                 UAVs_position(j1+1,1)=UAVs_position(1,1)-j1*x2;
                                 UAVs_position(j1+1,2)=UAVs_position(1,2)+j1*y2;
                             end
                            if rem(batch_UAV_num,2)~=0
                                for i1=1:((batch_UAV_num-1)/2)
                                    UAVs_position(i1+1+((batch_UAV_num-1)/2),1)=UAVs_position(1,1)-i1*x1;
                                    UAVs_position(i1+1+((batch_UAV_num-1)/2),2)=UAVs_position(1,2)+i1*y1;
                                end
                            else
                                for i1=1:floor((batch_UAV_num-1)/2)
                                    UAVs_position(i1+1+(batch_UAV_num/2),1)=UAVs_position(1,1)-i1*x1;
                                    UAVs_position(i1+1+(batch_UAV_num/2),2)=UAVs_position(1,2)+i1*y1;
                                end
                            end
                         else
                             for j1=1:(batch_UAV_num/2)
                                 UAVs_position(j1+1,1)=UAVs_position(1,1)+j1*x1;
                                 UAVs_position(j1+1,2)=UAVs_position(1,2)+j1*y1;
                             end
                              if rem(batch_UAV_num,2)~=0
                                  for i1=1:((batch_UAV_num-1)/2)
                                      UAVs_position(i1+1+((batch_UAV_num-1)/2),1)=UAVs_position(1,1)+i1*x2;
                                      UAVs_position(i1+1+((batch_UAV_num-1)/2),2)=UAVs_position(1,2)+i1*y2;
                                  end
                              else
                                  for i1=1:floor((batch_UAV_num-1)/2)
                                      UAVs_position(i1+1+(batch_UAV_num/2),1)=UAVs_position(1,1)+i1*x2;
                                      UAVs_position(i1+1+(batch_UAV_num/2),2)=UAVs_position(1,2)+i1*y2;
                                  end
                              end
                         end
                     end
                  end
             end
             update_positions=zeros(batch_UAV_num,2);
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
             end

            t_com=size(T_sweep,1)+2*t_up;
            T_com=[T_com;t_com];
 
            for i3=1:size(T_sweep,2)
                 input=1;
                 temp=T_sweep(:,i3)';
                 t=temp==input;
                out=find(diff([t false])==-1)-find(diff([false t])==1)+1;
                if (length(out(out>=Tcon)))>=1
                    sweep_success(i3,1)=1;
                end
            end  
                m1=0;
            if flag2==1
                n_start=size(T_sweep,1);
                 Time_start=[ Time_start;n_start];
                change_batch=[change_batch;size(T_sxy,1)];
                success_point=find(sweep_success);
                [calculate,~]=ismember(success_point,AOI_point);
                index_c=find(calculate==0);
                success_point1=success_point(index_c,1);
                %% AoI
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
                    Time_diff=size(T_sweep,1)-table(size(table,1),1)+2*t_up;
                    AOI=AOI+Time_diff;
                    Table{m1,1}=table;
                end  
               if flag_miss==0
                 ind1=find(sweep_success);
                 if k<2
                  for i4=1:size(cluster_member2) 
                      if judge_flag(i4,1)==0
                          member=cluster_member2{i4,:};
                          [~,e6]=ismember(member,ind1);
                          d6= all(e6(:));
                          if d6==1
                              judge_POI=seg_POI1(i4,:);
                              [~,e5]=ismember(seg_POI1,judge_POI);
                              d5=find(e5(:,1)~=0&e5(:,2)~=0&e5(:,3)~=0);
                              if d5~=0
                                  judge_flag([d5],:)=1;
                                  [~,d55]=find( Best_Route_new==d5);
                                 Best_Route_new(:,[d55])=[];
                                 n1=n1-1;
                              end
                          end
                      end
                  end
                 else
                  for i4=1:size(sigle_member1)
                      if  judge_flag(i4,1)~=1
                     member=sigle_member1(i4,:);
                     [~,e6]=ismember(member,ind1);
                     d6= all(e6(:)); 
                     if  d6==1
                         judge_POI=seg_POI1(i4,:);
                         [~,e5]=ismember(seg_POI1,judge_POI);
                         d5=find(e5(:,1)~=0&e5(:,2)~=0&e5(:,3)~=0);
                       
                     if d5~=0
                         judge_flag([d5],:)=1;
                          [~,d55]=find( Best_Route_new==d5);
                          Best_Route_new(:,[d55])=[];
                         n1=n1-1;
                     end
                     end
                      end
                 end   
                     
                 end
            end  
            end
    end

    if k<2
           judge_cluster_sweep=find(sweep_success);
            [~,d2]=ismember(cluster_ME,judge_cluster_sweep);
            index=find(d2==0);
            if isempty(index)~=1
                need_to_sweep=POI(cluster_ME(index,:),:);
                need_to_sweep_T=T(cluster_ME(index,:),1);
                seg_POI1=need_to_sweep;
                seg_POI1(:,3)=H_fly;
                seg_T1=need_to_sweep_T;
                flag_miss=1;
                sweep_missed=[sweep_missed;size(V_final,1)];
            else
                flag_miss=0;
            end
    else
          judge_sigle_sweep=find(sweep_success);
            [~,d2]=ismember(sigle_member,judge_cluster_sweep);
            index=find(d2==0);
            if isempty(index)~=1
                need_to_sweep=POI(sigle_member(index,:),:);
                need_to_sweep_T=T(sigle_member(index,:),1);
                seg_POI1=need_to_sweep;
                seg_POI1(:,3)=H_fly;
                seg_T1=need_to_sweep_T;
                flag_miss=1;
                sweep_missed=[sweep_missed;size(V_final,1)];
            else
                flag_miss=0;
            end
        
        
    end
 end
    
 for i6=1:size(T_sweep,2)
    a2=T_sweep(:,i6);
    b2=find(a2);
    res=[];
    table=[];
    n3=1;
    k2=1;
    while k2<size(b2,1)
        j6 = k2+1 ;
        while j6 <= size(b2,1) &  b2(j6)==b2(j6-1)+1
            n3 = n3 + 1 ;
            j6 = j6 + 1 ;
        end
    if n3 >= Tcon
        res = [res ; b2(k2),n3] ;
        table=[table;res];
    end
    n3 = 1 ;
    k2 = j6 ;
    end
    table=unique(table,'rows','stable');
    Table{i6,1}=table;
 end
 AOI_average=AOI/size(POI,1);
gobal_perid=size(T_sweep,1);

