%{
% Function:  Fast minimum bounding-box filter (FMBBF)
% Version:   V2.0.0
% Designer:  Jingneng Fu
% Reference: J. Fu et al., Small bounding-box filter for small target detection, Opt.Eng. 60(3), 033107(2021).
% Suggestion:Further optimization including k value, sizes and shapes of bounding boxes.
% Date:      2020-12-03
% Input:     ImgIn should be a matrix of double type,and k = 1.25 could be a good chioce.
%}
function ImgOut = FastMinBoundBoxFilter(ImgIn,k)
%
SigLab = zeros(size(ImgIn));
ImgOut = SigLab;
% Sizes of bouding boxes are 5*5-13*13
ConGapLow  = 2;     
ConGapHigh = 6; 
CellSize   = 9; 
HalfSize   = 4;
%
k1 = k+0.5;
k2 = k-0.5;
%
MagSize = 4*HalfSize;
ImgIn = padarray(ImgIn,[MagSize,MagSize],'symmetric');
[RowSize,ColSize] = size(ImgIn);
% Matrix of threshold
Tm = zeros(RowSize,ColSize);
% Matrix of mid-range
Mu = zeros(RowSize,ColSize);
%
for row = (MagSize+1):CellSize:(RowSize-MagSize)
    for col = (MagSize+1):CellSize:(ColSize-MagSize)    
        % An image is divided into blocks
        ImgTemp = ImgIn(row:(row+CellSize-1),col:(col+CellSize-1));  
        % Calculate the position of the maximum gray pixel in each block
        [RowLab,ColLab] = find(ImgTemp == max(ImgTemp(:)));
        RowLab = row -1 + RowLab(1);
        ColLab = col -1 + ColLab(1);
        % Cross the border of image domain
        if(RowLab<2*MagSize||ColLab<2*MagSize||RowLab>RowSize-2*MagSize||ColLab>ColSize-2*MagSize)
            continue;
        end
        % Renew Threshold
        if(Tm(RowLab,ColLab)==0) 
            ThresMin = inf;
            for BaGdGap = ConGapLow:ConGapHigh
                % Pixels in a bounding box
                PixBaGd = [ImgIn(RowLab-BaGdGap,(ColLab-BaGdGap):(ColLab+BaGdGap)),...
                           ImgIn(RowLab+BaGdGap,(ColLab-BaGdGap):(ColLab+BaGdGap)),...
                           ImgIn((RowLab-BaGdGap+1):(RowLab+BaGdGap-1),ColLab-BaGdGap)',...
                           ImgIn((RowLab-BaGdGap+1):(RowLab+BaGdGap-1),ColLab+BaGdGap)'];  
                BoxPixMax = max(PixBaGd);
                BoxPixMin = min(PixBaGd);
                % Threshold of FMBBF
                ThresTemp = k1*BoxPixMax-k2*BoxPixMin;
                %
                if(ThresMin>ThresTemp)
                    ThresMin = ThresTemp;
                    % Mid-range
                    m       = (BoxPixMax+BoxPixMin)/2;
                end       
            end     
            Tm(RowLab,ColLab) = ThresMin;
            Mu(RowLab,ColLab) = m;
        end  
        % The center of the small target may be in the block
        if(ImgIn(RowLab,ColLab)>Tm(RowLab,ColLab))   
            for row_ = (RowLab-HalfSize):(RowLab+HalfSize)
                for col_ = (ColLab-HalfSize):(ColLab+HalfSize)
                    ImgTemp = ImgIn((row_-HalfSize):(row_+HalfSize),(col_-HalfSize):(col_+HalfSize));  
                    % Calculate the position of the maximum gray pixel in new block
                    [RowLab_,ColLab_] = find(ImgTemp == max(ImgTemp(:)));
                    RowLab_ = row_ - HalfSize -1 + RowLab_(1);
                    ColLab_ = col_ - HalfSize -1 + ColLab_(1);
                    % Renew Threshold
                    if(Tm(RowLab_,ColLab_)==0) 
                        ThresMin = inf;
                        for BaGdGap = ConGapLow:ConGapHigh
                            PixBaGd = [ImgIn(RowLab_-BaGdGap,(ColLab_-BaGdGap):(ColLab_+BaGdGap)),...
                                       ImgIn(RowLab_+BaGdGap,(ColLab_-BaGdGap):(ColLab_+BaGdGap)),...
                                       ImgIn((RowLab_-BaGdGap+1):(RowLab_+BaGdGap-1),ColLab_-BaGdGap)',...
                                       ImgIn((RowLab_-BaGdGap+1):(RowLab_+BaGdGap-1),ColLab_+BaGdGap)'];  
                            BoxPixMax = max(PixBaGd);
                            BoxPixMin = min(PixBaGd);
                            ThresTemp = k1*BoxPixMax-k2*BoxPixMin;
                            %
                            if(ThresMin>ThresTemp)
                                ThresMin = ThresTemp;
                                m        = (BoxPixMax+BoxPixMin)/2;
                            end       
                        end     
                        %
                        Tm(RowLab_,ColLab_) = ThresMin;
                        Mu(RowLab_,ColLab_) = m;
                    end        
                    SigLab(row_-MagSize,col_-MagSize) = ImgIn(row_,col_)- Tm(RowLab_,ColLab_);
                    ImgOut(row_-MagSize,col_-MagSize) = ImgIn(row_,col_)- Mu(RowLab_,ColLab_);
                end
            end            
        end
    end
end
% Result
ImgOut = (SigLab>0).*ImgOut;
