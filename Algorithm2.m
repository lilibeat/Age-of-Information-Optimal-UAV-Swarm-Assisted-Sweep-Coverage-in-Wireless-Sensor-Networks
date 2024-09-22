
Re_UAV=250
Hmin=100;
H=150;
td=30;%the sweep period difference
R_UAV=sqrt(Re_UAV^2-H^2);
lammda1=0.7; 
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
 for  i=1:1:size(POI,1)
     text(x(i)+0.8,y(i)+0.8,z(i)+0.8, num2str(i))
 end
T1=T;
POI1=POI;
k=1;
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
no_add=[];
no_add_T=[]
while isempty(POI1)~=1
    for i=1
        D_POI1=[];
        sign=zeros(size(POI1,1),1);
        sign(i,1)=1;
        for j=2:size(POI1,1)
            D_POI1(i,j)=sqrt(sum((POI1(i,:)-POI1(j,:)).^2));
            T_dif=abs(T1(i,1)-T1(j,1));
            if D_POI1(i,j)<2*R_UAV&&T_dif<td
                if sign(j,1)~=0&&sign(j,1)<sign(i,1)
                    sign(i,1)=sign(j,1);
                else
                    sign(j,1)=sign(i,1);
                end
            end
        end
        temp=[];
        T_temp=[];
        H_temp=[];
        add_judge=[];
        add_T=[];
        add_H=[];
        k=1;
        R_UAV_change=R_UAV;
        H_new=H;
        for g=1:size(sign,1)
            if sign(g,1)==1
                T_temp=[T_temp;T1(g,1)];
                temp=[temp;POI1(g,1:3)];
                H_temp=[H_temp;POI1(g,3)];
            end
        end
        if size(temp,1)==1
            n=1;
            h=temp(1,3);
            if H>=Hmin+h
                H_fly=H;
            else
                H_fly=H+h;
            end
            t=T_temp;
            SPOI_number=[SPOI_number;n];%记录个数，便于后续变速
            sigle_H_fly=[sigle_H_fly;H_fly];
            sigle_POI=[sigle_POI;temp(:,1:3)];
            sigle_T=[sigle_T;T_temp];
            [~,ind]=ismember(temp,POI,'rows');
            sigle_member=[sigle_member;ind];
        else
            D_temp=[];
            for i=2:size(temp,1)
                for j=1:i-1
                       D_temp(i,j)=sqrt(sum((temp(i,:)-temp(j,:)).^2));
                  end
            end 
        temp_new=temp;
        T_temp_new=T_temp;
        H_temp_new=H_temp;
         D_temp(D_temp ==0)=NaN;
         m=min(D_temp);
         mm=min(m)
         [p1,p2]=find(D_temp==mm);
         temp_new(p1,1:3)=0;
         temp_new(p2,1:3)=0;
         id=temp_new(:,2)==0;
         temp_new(id,:)=[];
         T_temp_new(id,:)=[];
         H_temp_new(id,:)=[];
         r=sqrt(sum((temp(p1,1:2)-temp(p2,1:2)).^2))/2;
         cen=(temp(p1,1:2)+temp(p2,1:2))/2;
         A=vertcat(temp(p1,1:3),temp(p2,1:3));
         [~,ind]=ismember(A,POI,'rows');
         cluster_ME=[cluster_ME;ind];
         h=max(temp(:,3));
          for i=1:size(temp_new,1)
            newp=temp_new(i,1:2);    
            dc=sqrt(sum((cen(1,:)-newp(1,:)).^2));
            if dc>r
                r=(r+dc)/2;
                cen=cen+(dc-r)/dc*(newp-cen);
            end  
            if r>R_UAV_change
                add_judge(k,1:3)=temp_new(i,1:3);
                add_T(k,1)=T_temp_new(i,1);
                add_H(k,1)=H_temp_new(i,1)+H;
                k=k+1;
                R_change=ceil(r);
                H_change=sqrt(Re_UAV^2-R_change^2)
                if H_change>(Hmin+h)||H_change==(Hmin+h)
                    if H_change-(Hmin+h)>=10
                        H_change=H_change-10;
                    else
                        H_change=(Hmin+h);
                    end
                    H_new=ceil(H_change);
                    add_judge=[];
                    R_UAV_change=R_change;  
                else
                    no_add=[no_add;add_judge];
                    no_add_T=[no_add_T;add_T];
                    no_add_H=no_add(:,3)+H;
                   
                end
            end
          end
          if isempty(add_judge)==0
              temp_new=setdiff(temp_new,add_judge, 'rows');
          end
         [~,ind]=ismember(temp_new,POI,'rows');
         cluster_ME=[cluster_ME;ind];
         r=sqrt(sum((temp(p1,1:2)-temp(p2,1:2)).^2))/2; 
         cen=(temp(p1,1:2)+temp(p2,1:2))/2; 
         for i=1:size(temp_new,1)
            newp=temp_new(i,1:2);    
            dc=sqrt(sum((cen(1,:)-newp(1,:)).^2));
            if dc>r
                r=(r+dc)/2;
                cen=cen+(dc-r)/dc*(newp-cen);
            end
         end
        [~,e]=ismember(temp,add_judge);
        d=[];
        if isempty(e)
        else
            d=find(e(:,1)~=0);
            temp(d,:)=[];
            T_temp(d,:)=[];
        end    
        n=size(temp,1);
        h=max(temp(:,3));
       if H_new>=Hmin+h
            H_fly=H_new;
        else
           H_fly=H_new+h;
        end

        t=min(T_temp);
        CPOI_number=[CPOI_number;n];
        cluster_H_fly=[cluster_H_fly;H_fly];
        cluster_r=[cluster_r;r];
        cluster_T=[cluster_T;t];
        cen=[cen h];
        cluster_POI=[cluster_POI;cen];
        end
        [~,e1]=ismember(POI1,temp);
        if isempty(e1)
        else
            d1=find(e1(:,1)~=0&e1(:,2)~=0);
            POI1(d1,:)=[];
            T1(d1,:)=[];
        end
    end
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

if isempty(sigle_POI)
else
flag=zeros(1,size(sigle_POI,1));
del_sigle_POI=[];
for i=1:size(cluster_POI,1)
    for j=1:size(sigle_POI,1)
        if flag(1,j)==0
            D_sc=sqrt(sum((cluster_POI(i,:)-sigle_POI(j,:)).^2));
            r=(r+D_sc)/2;
            cen=cluster_POI(i,1:2)+(dc-r)/dc*(sigle_POI(j,1:2)-cluster_POI(i,1:2));
            temp_cluster=[cluster_POI(i,:);sigle_POI(j,:)];
            h=max(temp_cluster(:,3));
            Hsafe=Hmin+h;
            H_change=sqrt(Re_UAV^2-r^2);
            if H_change>Hsafe
                del_sigle_POI=[del_sigle_POI;sigle_POI(j,:)];
                H_fly=H_change+h;
                cluster_POI(i,1:2)=cen;
                cluster_H_fly(i,1)=H_fly;
                cluster_POI(i,3)=H_fly;
                t=min(cluster_T(i,1),sigle_T(j,1));
                cluster_T(i,1)=t;
                [~,ind]=ismember(sigle_POI(j,:),POI,'rows');
                add_member=[cell2mat(cluster_member(i,:)) ind];
                cluster_member{i,:}=add_member;%
                CPOI_number(i,1)=size(cluster_POI,1)
            end
        end   
    end
end
[~,e]=ismember(sigle_POI,del_sigle_POI);
if isempty(e)
    sigle_POI(e(:,1),:)=[];
    sigle_T(e(:,1),:)=[];
    SPOI_number(e(:,1),:)=[];
    sigle_H_fly(e(:,1),:)=[];
end
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
start_r=[240];

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


