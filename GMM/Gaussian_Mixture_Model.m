function [varargout] = Gaussian_Mixture_Model(action,varargin)
%GMM 
%initialization function is used to definite initial parameters such as
%means variance weights and number of Gaussian Model
%Plot function show every Gaussian Model's zone
global GMM
switch lower(action)
    case 'initialization'
        GMM.dataset = varargin{1};
        GMM.means = varargin{2};
        GMM.vars = varargin{3};
        GMM.alpha = varargin{4};
        GMM.numGM = varargin{5}; 
        [NumData, Dimension] = size(GMM.dataset);
        GMM.NumData = NumData;
        GMM.Dimension = Dimension;
        GMM.labels = zeros(GMM.NumData,1);
        
    case 'train'
        iteration = varargin{1};
        GMM.Gamma = zeros(GMM.NumData,GMM.numGM);
        
        for i=1:iteration
    
            for j=1:GMM.NumData
                for k = 1:GMM.numGM
                    GMM.Gamma(j,k) = GMM.alpha(k)*Normal_Distribution(GMM.means(k,:),GMM.vars(:,:,k),GMM.dataset(j,:));
                end
                GMM.Gamma(j,:) = GMM.Gamma(j,:)/sum(GMM.Gamma(j,:));
            end
    
            for k = 1:GMM.numGM
                GMM.vars(:,:,k) = (GMM.dataset-GMM.means(k,:))'*(GMM.Gamma(:,k).* (GMM.dataset-GMM.means(k,:)))/sum(GMM.Gamma(:,k));
                GMM.means(k,:) = sum(GMM.Gamma(:,k).* GMM.dataset,1) / sum(GMM.Gamma(:,k));
                GMM.alpha(k) = sum(GMM.Gamma(:,k))/GMM.NumData;
            end
    
            [maxValue,labels] = max(GMM.Gamma,[],2); 
            times = 0;
            if sum(abs(GMM.labels - labels)) > 0
                times = 0;
                GMM.labels = labels;
            else
                times = times + 1;
                if times ==10
                    break
                end
            end
        end  
        
     varargout(1) = {GMM.means};
     varargout(2) = {GMM.vars};
     varargout(3) = {GMM.alpha};
     
    case 'plot'
       %% plot GMM
        nbDrawingSeg = 40;
        t = linspace(-pi, pi, nbDrawingSeg)';
        figure('NumberTitle', 'off', 'Name','GMM encoding results','position',[100,200,800,300])
        subplot(2,2,1);hold on;
        for j=1:GMM.numGM
            stdev = sqrtm(3.0.*GMM.vars([1,2],[1,2],j));
            X = [cos(t) sin(t)] * real(stdev) + repmat(GMM.means(j,[1,2]),nbDrawingSeg,1);
            patch(X(:,1), X(:,2), [0.6,1,0.6], 'lineWidth', 2, 'EdgeColor', [0 0.8 0]);hold on;
            plot(GMM.means(:,1), GMM.means(:,2), 'x', 'lineWidth', 2, 'color', [0 0.8 0]);hold on;
        end
        axis([min(GMM.dataset(:,1)) max(GMM.dataset(:,1)) min(GMM.dataset(:,2))-0.01 max(GMM.dataset(:,2))+0.01]);
        xlabel('t','fontsize',16); ylabel('X_{position}','fontsize',16);

        subplot(2,2,3);hold on;
        for j=1:GMM.numGM
            stdev = sqrtm(3.0.*GMM.vars([1,3],[1,3],j));
            X = [cos(t) sin(t)] * real(stdev) + repmat(GMM.means(j,[1,3]),nbDrawingSeg,1);
            patch(X(:,1), X(:,2), [0.6,1,0.6], 'lineWidth', 2, 'EdgeColor', [0 0.8 0]);hold on;
            plot(GMM.means(:,1), GMM.means(:,3), 'x', 'lineWidth', 2, 'color', [0 0.8 0]);hold on;
        end
        axis([min(GMM.dataset(:,1)) max(GMM.dataset(:,1)) min(GMM.dataset(:,3))-0.01 max(GMM.dataset(:,3))+0.01]);
        xlabel('t','fontsize',16); ylabel('Y_{position}','fontsize',16);

        subplot(2,2,[2,4]);hold on;
        for j=1:GMM.numGM
            stdev = sqrtm(3.0.*GMM.vars([2,3],[2,3],j));
            X = [cos(t) sin(t)] * real(stdev) + repmat(GMM.means(j,[2,3]),nbDrawingSeg,1);
            patch(X(:,1), X(:,2), [0.6,1,0.6], 'lineWidth', 2, 'EdgeColor', [0 0.8 0]);hold on;
            plot(GMM.means(:,2), GMM.means(:,3), 'x', 'lineWidth', 2, 'color', [0 0.8 0]);hold on;
        end
        axis([min(GMM.dataset(:,2)) max(GMM.dataset(:,2)) min(GMM.dataset(:,3))-0.01 max(GMM.dataset(:,3))+0.01]);
        xlabel('X_{position}','fontsize',16); ylabel('Y_{position}','fontsize',16);
        hold off ;
        drawnow;
end