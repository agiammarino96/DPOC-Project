mG=G;
load('exampleG.mat')
con=0;
K=length(stateSpace);
tol=0.0001;
L=5;
v=zeros(K,L);
for u=1:L
    for k=1:K
        if mG(k,u)==Inf && G(k,u)==Inf
            mG(k,u)=0;
            G(k,u)=0;
        end
        if abs(mG(k,u)-G(k,u))>tol
            con=1;
            v(k,u)=abs(mG(k,u)-G(k,u));
        end
        
    end
    
end
