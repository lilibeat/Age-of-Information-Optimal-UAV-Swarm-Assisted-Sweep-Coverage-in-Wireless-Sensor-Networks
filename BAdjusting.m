
function [Land2]=BAdjusting(Population2,numButterfly2,Best_Route)

Land2=Population2;
L=length(Population2(1,:));%���� Population2��Ԫ��
partition=5/12;
for i=1:numButterfly2
    if rand<=partition
        Land2(i,:)=Best_Route;% ��Ŀǰ��õĸ��帳����Land2(i,:),Ȼ���Land2(i,:)����
                              % �����ϣ��൱�ڽ���õĽⲿ�ָ�����һ��
        r=unidrnd(L,[1,2]);   %������34�α���
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
          %a = round(rand(1,2)*(N-1) + 1); %rand(1,2)����1��2��0��1ֱ�ӵ������
          a = unidrnd(N,1,2); %unidrnd(n)ֱ�Ӳ���n���ڵ������
          S_left = Population2(i,1:min(a)-1);
          S_mid = fliplr(Population2(i,min(a):max(a))); %���ҷ�ת�м�ľ���(����ѡ�е����������к�����֮������г���)
          S_right = Population2(i,max(a)+1:N);
          Land2(i,:) = [S_left,S_mid,S_right];
    end








end

    
end