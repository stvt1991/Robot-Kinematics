function [T0Tn,entities] = DenaHart(alpha, d, theta, r)
%% Creator: Swaminath Venkateswaran, ESILV Engineering School, Paris, France %%
%% A function 'DenaHart.m' to compute the homogeneous transformation matrix of a given serial robot
% INPUT(S)     : alpha,d ,theta,r - The four modified DH parameters (in the form of a row vector)
% OUTPUT(S)    : T0Tn - The resultant homogenous transformation matrix, entities - The intermediate transformation matrices
%%

entities = struct();

n = length(alpha);


    for i = 1:(n-1)
        i
        if(i==1)
            T1= DHMatrix(i)
            T2= DHMatrix(i+1)
            T0Tn= simplify(T1*T2)
            entities(i).ele= T1;
            entities(i+1).ele= T0Tn;
        else
            T2= DHMatrix(i+1)
            T0Tn= simplify(T0Tn*T2)
            entities(i+1).ele= T0Tn;
        end
    end


    function T = DHMatrix(j)

        Ct = cos(theta(j));
        St = sin(theta(j));
        Ca = cos(alpha(j));
        Sa = sin(alpha(j));
        
        T = [Ct, -St, 0, d(j);
            Ca*St, Ca*Ct, -Sa, -r(j)*Sa;
            Sa*St, Sa*Ct, Ca, r(j)*Ca; 
            0, 0, 0, 1];

    end

end



