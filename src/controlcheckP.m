mP=P;
load('exampleP.mat')
con=0;
K=length(stateSpace);
tol=0.0001;
L=5;
v=zeros(K,K,L);
for u=1:L
    for k=1:K
        for t=1:K
            
           if abs(mP(k,t,u)-P(k,t,u))>tol
                con=1;
                v(k,t,u)=abs(mG(k,u)-G(k,u));
            end
        end
    end
    
end
