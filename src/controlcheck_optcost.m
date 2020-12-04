global K
check_opt_cost=1;
J_opt_lp=round(J_opt_lp,4);
J_opt_pi=round(J_opt_pi,4);
J_opt_vi=round(J_opt_vi,4);
for k=1:K
   if J_opt_lp(k)~=J_opt_pi(k)
       check_opt_cost=0;
       disp('Linear Programming cost does not match with Policy iteration cost')
       break
   end
   if J_opt_lp(k)~=J_opt_vi(k)
       check_opt_cost=0;
       disp('Linear Programming cost does not match with Policy iteration cost')
       break
   end
end
if check_opt_cost==1
    disp('Optimal cost check succeded')
end