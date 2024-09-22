function Best_Route =imbo_route(sink,batch_T,T_high1)
            t0=clock;
    x = sink(:,1);
    y= sink(:,2);
    plot(x,y,'.k');
    xlabel('X');      
    ylabel('Y');       
    title('IMBO');    
    grid on
    n1 = size(sink,1);
D1=zeros(n1,n1);
for i = 1:n1
    for j = 1:n1    
            D1(i,j) = sqrt((x(i)-x(j))^2 + (y(i)-y(j))^2);  
    end    
end
%% Initial parameter setting
OPTIONS.popsize =50; % total population size 
OPTIONS.Maxgen =1000; % generation count limit
OPTIONS.numVar =n1; % number of vriables in each population member 
OPTIONS.partition = 5/12;  % the percentage of population for MBO
%Keep = 2; % elitism parameter: how many of the best habitats to keep from one generation to the next
maxStepSize = 1.0;        %Max Step size
partition = OPTIONS.partition;  % OPTIONS.partition=5/12 
numButterfly1 = ceil(partition*OPTIONS.popsize);  % NP1 in paper, OPTIONS.popsize=50£¬numButterfly1
numButterfly2 = OPTIONS.popsize - numButterfly1; % NP2 in paper
period = 1.2; % 12 months in a year£¬
Land1 = zeros(numButterfly1, OPTIONS.numVar); 
Land2 = zeros(numButterfly2, OPTIONS.numVar);
BAR = partition; % you can change the BAR value in order to get much better performance

Route_up=zeros(OPTIONS.Maxgen,n1+1);
Best_value=zeros(OPTIONS.Maxgen,1);
Fitness=zeros(OPTIONS.Maxgen,1);
Length_ave=zeros(OPTIONS.Maxgen,1);
Route_long = zeros(1,1);

serial_T1=zeros(1,n1+1);
serial_T=zeros(1,n1);
serial_T(1,1:n1)=linspace(1,size(batch_T,1),size(batch_T,1));
serial_T1(1,1:n1)=linspace(1,size(batch_T,1),size(batch_T,1));
global Chrom
Chrom=zeros(OPTIONS.popsize,n1);
Ch_T=zeros(OPTIONS.popsize,n1);
for i1=1:OPTIONS.popsize
   Chrom(i1,:)=randperm(n1);
end
for i1=1:OPTIONS.popsize
    S=Chrom(i1,:);
    for i2=1:size(S,2);
       Ch_T(i1,i2)=T_high1(S(1,i2));
    end
end
for i3=1:OPTIONS.Maxgen
    [Population1,Population2,Best_Route] = PopSort( serial_T,T_high1,D1,Chrom,numButterfly1,numButterfly2);
    Route_up(i3,1:n1)=Best_Route;     %
    Route_up(i3,n1+1)=Best_Route(1);
    Best_value(i3) = T_R_diff( serial_T1,T_high1,D1,Route_up(i3,:));
    Fitness(i3)=-1/Best_value(i3);
    [Land1]=Migration(Population1,Population2,numButterfly1);
     for kk=1:numButterfly1         % 
         if T_R_diff( serial_T,T_high1,D1,Land1(kk,:))<T_R_diff( serial_T,T_high1,D1,Population1(kk,:))
             Land1(kk,:)=Population1(kk,:);   
        end
     end
    [Land2]=BAdjusting(Population2,numButterfly2,Best_Route);
     for kkk=1:numButterfly2         %
         if T_R_diff( serial_T,T_high1,D1,Land2(kkk,:))<T_R_diff( serial_T,T_high1,D1,Population2(kkk,:))
            Land2(kkk,:)=Population2(kkk,:);   
        end
     end
      NewChrom=Land1;
    for i=1:numButterfly2
        NewChrom(i+numButterfly1,:)=Land2(i,:);
    end
    Chrom=NewChrom;
end
Time_Cost=etime(clock,t0)
for i=1:OPTIONS.popsize
    T_Rlong = T_R_diff( serial_T,T_high1,D1,Chrom(i,:));
end
[a,b]=max(T_Rlong);
Best_Route=Chrom(b,:);

end

