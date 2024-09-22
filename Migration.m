function [Land1]=Migration(Population1,Population2,numButterfly1)
period=1.2;
partition=5/12;
Land1=Population1;

for i=1:numButterfly1
     r1=rand*period;
    if r1<=partition           %
          N= length(Population1(1,:));
          a = unidrnd(N,1,2); %unidrnd(n)       
          S_left = Population1(i,1:min(a)-1);
          S_mid = fliplr(Population1(i,min(a):max(a))); %Flip 
          S_right = Population1(i,max(a)+1:N);
          Land1(i,:) = [S_left,S_mid,S_right];
        
         
    else
        L=length(Population2(i,:));      
        r=unidrnd(L,[1,2]);
        w1=Population2(i,r(1));
        Population2(i,r(1))=Population2(i,r(2));
        Population2(i,r(2))=w1;
        
        r=unidrnd(L,[1,2]);
        w1=Population2(i,r(1));
        Population2(i,r(1))=Population2(i,r(2));
        Population2(i,r(2))=w1;
        Land1(i,:)=Population2(i,:);
    end  
end







