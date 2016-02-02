function [ cost ] = mutual_cost(current_performance,desired_performance,importance,Nc,Nd)


series=zeros(Nd,Nc);

for i=1:Nd
    for j=1:Nc
        series(i,j)=norm(current_performance(j,:)-desired_performance(i,:));
    end
end

minimums=min(series);

cost=dot(minimums,importance);







% % % 
% % % series=zeros(Nd,Nc,Nchar);
% % % 
% % % for i=1:Nd
% % %     for j=1:Nc
% % % %         series1(i,j)=norm((current_performance(j,:)-desired_performance(i,:)));
% % %         for k=1:Nchar
% % %             series(i,j,k)=(current_performance(j,k)-desired_performance(i,k));
% % %         end
% % %     end
% % % end
% % % series=abs(series);
% % % 
% % % for k=1:Nchar
% % %     max_delta_char=max(max(series(:,:,k)));
% % % end
% % % 
% % % for i=1:Nd
% % %     for j=1:Nc
% % %        s1=dot(series(i,j,:),max_delta_char);
% % %     end
% % % end
% % % 
% % % minimums=min(series1)
% % % % 
% % % cost=dot(minimums,importance);



end

