
Re_UAV=250
Hmin=100;%
H=150;
td=30;
R_UAV=sqrt(Re_UAV^2-H^2);
lammda1=0.4;
figure(1)
x=POI(:,1);
y=POI(:,2);
z=POI(:,3);
plot3(x,y,z,'bo');
grid on
hold on
set(gca,'ylim',[0,4000],'ytick',[0:200:4000]);
set(gca,'xlim',[0,4000],'xtick',[0:200:4000]);
set(gca,'zlim',[0,50],'ztick',[0:10:50]);
xlabel('x/m');
ylabel('y/m');
zlabel('z/m');
 for  j=1:1:size(POI,1)
     text(x(j)+0.8,y(j)+0.8,z(j)+0.8, num2str(j))
 end
T1=T;
POI1=POI;
j=1;
sigle_POI=[];
sigle_T=[];
sigle_member=[];
cluster_POI=[];
cluster_T=[];
cluster_ME=[];
T_temp=[];
cluster_r=[];
CPOI_number=[];
SPOI_number=[];
cluster_H_fly=[];
sigle_H_fly=[];
while isempty(POI1)~=1
    i=1;
    flag=1;
    it=1;
    sign=zeros(size(POI1,1),1);
    t1=POI1(i,1:3);
    temp=POI1(i,1:3);
    T_temp=[];
    H_temp=[];
    sign(1,1)=1;
    while flag
        for j=1:size(POI1,1)
            if sign(j,1)~=1
                if sqrt((POI1(j,1)-t1(1,1))^2+(POI1(j,2)-t1(1,2))^2)<2*R_UAV &&abs(T1(j,1)-T1(1,1))<=td;
                    temp=[temp;POI1(j,1:3)]; 
                    sign(j,1)=1;
                end
            end
        end
        a=find(sign~=0);
        if it==size(temp,1) || size(POI1,1)==size(find(sign~=0),1)
            flag=0;
        else
            it=it+1;
            t1=temp(it,:);
        end
    end
    [~,ind]=ismember(temp,POI,'rows');
    T_temp=[T_temp;T(ind,1)];
    H_temp=[H_temp;POI(ind,3)];
    add_judge=[];
    add_T=[];
    add_H=[];
        k=1;
        H_new=H;
     if size(temp,1)==1
            n=1;
            h=temp(1,3);
            if H>=Hmin+h
                H_fly=H;
            else
                H_fly=H+h;
            end
            t=T_temp;
            SPOI_number=[SPOI_number;n];
            sigle_H_fly=[sigle_H_fly;H_fly];
            sigle_POI=[sigle_POI;temp(:,1:3)];
            sigle_T=[sigle_T;T_temp];
            [~,ind]=ismember(temp,POI,'rows');
            sigle_member=[sigle_member;ind];
     else
            temp_new=temp;
            T_temp_new=T_temp;
            H_temp_new=H_temp;
            while isempty(temp_new)~=1
                D_temp_new=[];
                no_add=[];
                no_add_T=[];
                temp=temp_new;   
                T_temp=T_temp_new;
                H_temp=H_temp_new;
                if size(temp_new,1)<2
                    cen=temp(1,1:2);
                    if H>=Hmin+temp(1,3)
                        H_fly=H;
                    else
                        H_fly=H_new+temp(1,3);
                    end
                    r=R_UAV;
                    [~,ind]=ismember(temp,POI,'rows');
                    cluster_ME=[cluster_ME;ind];
                    temp_new=[];
                else
                for i=2:size(temp_new,1)
                    for j=1:i-1
                       D_temp_new(i,j)=sqrt(sum((temp_new(i,:)-temp_new(j,:)).^2));
                    end
                end
                D_temp_new(D_temp_new ==0)=NaN;
                [p1,p2]=find(D_temp_new==(min(min(D_temp_new))));
                p_sum=[p1;p2];
                temp_new(p_sum(:,1),:)=[];
                r=sqrt(sum((temp(p1,1:2)-temp(p2,1:2)).^2))/2;
                cen=(temp(p1,1:2)+temp(p2,1:2))/2;
                h=max(temp(:,3));
                for i=1:size(temp_new,1)
                    newp=temp_new(i,1:2);    
                    dc=sqrt(sum((cen(1,:)-newp(1,:)).^2));
                    if dc>r
                        r_new=(r+dc)/2;
                        if r_new<=R_UAV
                            cen=cen+(dc-r_new)/dc*(newp-cen);
                            r=r_new;
                        else
                            no_add=[no_add;temp_new(i,:)];
                        end
                    end
                end
                if isempty(no_add)==1
                    temp_new(:,1:3)=[];
                else
                    [~,e]=ismember(temp,no_add);
                    d=find(e(:,1)~=0);
                    temp_new=no_add;
                    T_temp_new=T_temp(d,1);
                    H_temp_new=H_temp(d,1);
                    temp(d,:)=[];
                    T_temp(d,:)=[];
                  
                    [~,e2]=ismember(POI1,temp);
                    d1=find(e2(:,1)~=0&e2(:,2)~=0);
                    POI1(d1,:)=[];
                    T1(d1,:)=[];
                end
                [~,ind]=ismember(temp,POI,'rows');
                cluster_ME=[cluster_ME;ind];
                h=max(temp(:,3));
                if H>=Hmin+h
                    H_fly=H;
                else
                    H_fly=H_new+h;
                end
                end
                CPOI_number=[CPOI_number;size(temp,1)];
                cluster_H_fly=[cluster_H_fly;H_fly];
                cluster_r=[cluster_r;r];
                cluster_T=[cluster_T;min(T_temp)];
                cen=[cen h];
                cluster_POI=[cluster_POI;cen];  
            end
     end
         [~,e2]=ismember(POI1,temp);
         d1=find(e2(:,1)~=0&e2(:,2)~=0);
         POI1(d1,:)=[];
         T1(d1,:)=[];
    end
cluster_member=cell(size(cluster_POI,1),1);
cluster_ME1=cluster_ME;
for i=1:size(cluster_member,1)
    num=CPOI_number(i,1);
    num_temp=cluster_ME1(1:num,1);
    cluster_ME1(1:num)=[];
    num_temp=num_temp';
    cluster_member{i,:}=num_temp;
end   
cluster_POI(:,3)=cluster_H_fly;
sigle_POI(:,3)=sigle_H_fly;
sigle_r=R_UAV*ones(size(sigle_POI,1),1);
for i=1:size(cluster_r,1)
    if cluster_r(i,1)>R_UAV
        cluster_r(i,1)=ceil(cluster_r(i,1));
    else
        cluster_r(i,1)=R_UAV;
    end
end

all_POI=cell(2,1);
all_T=cell(2,1);
all_r=cell(2,1)
for i=1:size(all_POI,1)
    if i==1
        all_POI{i,:}=cluster_POI;
        all_T{i,1}=cluster_T;
        all_r{i,1}=cluster_r;
    else
        all_POI{i,1}=sigle_POI;
        all_T{i,1}=sigle_T;
        all_r{i,1}=sigle_r;
    end
    
end

start=[1200 1200 H];
start_r=[R_UAV];

figure(2)
x=POI(:,1);
y=POI(:,2);
z=POI(:,3);
plot3(x,y,z,'bo');
grid on
hold on
xlabel('x/m');
ylabel('y/m');
zlabel('z/m');
 for  i=1:1:size(POI,1)
     text(x(i)+0.8,y(i)+0.8,z(i)+0.8, num2str(i))
 end

 for i=1:size(all_POI,1)
     seg_POI=cell2mat(all_POI(i));
     seg_r=cell2mat(all_r(i));
     seg_POI=[seg_POI;start];
     seg_r=[seg_r;start_r];
     x1=seg_POI(:,1);
     y1=seg_POI(:,2);
     plot(x1,y1,'r*');
     hold on
     for j=1:size(seg_POI,1)
         text(x1(j)+0.8,y1(j)+0.8, num2str(j));
         rectangle('Position',[seg_POI(j)-seg_r(j,1),seg_POI(j+size(seg_POI,1))-seg_r(j,1),2*seg_r(j,1),2*seg_r(j,1)],'Curvature',[1,1],'edgecolor','r'),axis equal;%圆心坐标为(20,30)，半径为10,轮廓颜色为红色
     end
     grid on
     hold on   
 end
hold on
set(gca,'ylim',[0,4000],'ytick',[0:200:4000]);
set(gca,'xlim',[0,4000],'xtick',[0:200:4000]);
set(gca,'zlim',[0,20],'ztick',[0:2:20]);
  
    
    
    
    
    
    

     
        
       
            
        

    
    
    
    
    
    
    
    
    

     
        
       
            
        

    
    
    