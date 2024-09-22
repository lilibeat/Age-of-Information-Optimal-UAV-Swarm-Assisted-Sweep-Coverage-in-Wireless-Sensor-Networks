
function [Land2]=BAdjusting(Population2,numButterfly2,Best_Route)

Land2=Population2;
L=length(Population2(1,:));%计算 Population2的元素
partition=5/12;
for i=1:numButterfly2
    if rand<=partition
        Land2(i,:)=Best_Route;% 将目前最好的个体赋予了Land2(i,:),然后对Land2(i,:)变异
                              % （接上）相当于将最好的解部分赋予下一代
        r=unidrnd(L,[1,2]);   %连续来34次变异
        w1=Land2(i,r(1));
        Land2(i,r(1))=Land2(i,r(2));
        Land2(i,r(2))=w1;
        
        r=unidrnd(L,[1,2]);
        w1=Land2(i,r(1));
        Land2(i,r(1))=Land2(i,r(2));
        Land2(i,r(2))=w1;
        
        r=unidrnd(L,[1,2]);
        w1=Land2(i,r(1));
        Land2(i,r(1))=Land2(i,r(2));
        Land2(i,r(2))=w1;
        
        r=unidrnd(L,[1,2]);
        w1=Land2(i,r(1));
        Land2(i,r(1))=Land2(i,r(2));
        Land2(i,r(2))=w1;
        
         r=unidrnd(L,[1,2]);
        w1=Land2(i,r(1));
        Land2(i,r(1))=Land2(i,r(2));
        Land2(i,r(2))=w1;

        
    else
%         r=unidrnd(L,[1,2]);
%         w1=Population2(i,r(1));
%         Population2(i,r(1))=Population2(i,r(2));
%         Population2(i,r(2))=w1;
%         Land2(i,:)=Population2(i,:);
          N = length(Population2(1,:));
          %a = round(rand(1,2)*(N-1) + 1); %rand(1,2)产生1行2列0和1直接的随机数
          a = unidrnd(N,1,2); %unidrnd(n)直接产生n以内的随机数
          S_left = Population2(i,1:min(a)-1);
          S_mid = fliplr(Population2(i,min(a):max(a))); %左右翻转中间的矩阵(包括选中的这两个城市和它们之间的所有城市)
          S_right = Population2(i,max(a)+1:N);
          Land2(i,:) = [S_left,S_mid,S_right];
    end








end

    
end