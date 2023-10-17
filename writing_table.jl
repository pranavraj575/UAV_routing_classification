using Printf

function write_table_head(f)
    rows="3"
    write(f,"\\begin{longtable}{|c|c|c|c|c|c|c|c|}\\hline\n")
    write(f,"\t\\multirow{"*rows*"}{*}{\\textbf{Instance}} ")
    write(f,"& \\multirow{"*rows*"}{*}{\$\\alpha\$ (s\\textsuperscript{-1})} ")
    write(f,"& \\multirow{"*rows*"}{*}{\$\\tau\$ (s)} ")
    write(f,"& {\\textbf{Initial Obj}} ")
    write(f,"& {\\textbf{Local Obj}} ")
    write(f,"& {\\textbf{Perturb Obj}} ")
    write(f,"& {\\textbf{Dwell}} ")
    write(f,"& {\\textbf{Compute}} ")
    write(f,"\\\\")
    write(f,"\n\t ")
    write(f,"& ")
    write(f,"& ")
    write(f,"& {\\textbf{(Tour Cost}} ")
    write(f,"& {\\textbf{(Tour Cost}} ")
    write(f,"& {\\textbf{(Tour Cost}} ")
    write(f,"& {\\textbf{Times}} ")
    write(f,"& {\\textbf{Times\$^{\\mathrm{\\boldsymbol{a}}}\$ (s)}} ")
    write(f,"\\\\")
    write(f,"\n\t ")
    write(f,"& ")
    write(f,"& ")
    write(f,"& {\\textbf{Range)}} ")
    write(f,"& {\\textbf{Range)}} ")
    write(f,"& {\\textbf{Range)}} ")
    write(f,"& {\\textbf{Range}} ")
    write(f,"& ")
    write(f,"\\\\")
    write(f,'\n')
    write(f,"\t\\hline \\hline")
    write(f,'\n')

end
function real_round(n,digits)
    return round(n*10^digits)/10^(digits)
end
function write_table_entry(f,identifier,alpha,tau,initials,locals,perturbations,dwell_times,computation_times,rounding=2)
    #rows=max(length(dwell_times),length(computation_times))
    rows=2
    multirow_cmd="\\multirow{"*string(rows)*"}{*}"
    function rnd(value,place=rounding)
        return string(real_round(value,place))
    end
    function multirow_round(value,place=rounding)
        return multirow_cmd*"{"*rnd(value,place)*"}" 
    end
    write(f,'\t')

    write(f,multirow_cmd*"{"*string(identifier)*"}")
    write(f," & ")
    write(f, multirow_cmd*"{"*(@sprintf "%.2e" alpha)*"}")
    write(f," & ")
    write(f,multirow_round(tau))
    write(f," & ")
    write(f,rnd(initials[1],rounding))
    write(f," & ")
    write(f,rnd(locals[1]))
    write(f," & ")
    write(f,rnd(perturbations[1]))
    write(f," & ")
    write(f,string(real_round(minimum(dwell_times),rounding)))
    write(f," & ")
    write(f,string(real_round(computation_times[1],rounding)))
    for time in computation_times[2:length(computation_times)]
        write(f," \$\\mid\$ ")
        write(f,string(real_round(time,rounding)))
    end
    write(f,"\\\\\n")
    for i in 2:rows
        write(f,'\t')
        write(f," & ")
        write(f," & ")
        write(f," & ")    
        write(f,"("*rnd(minimum(initials[2]),rounding)*", "*rnd(maximum(initials[2]),rounding)*")")
        write(f," & ")
        write(f,"("*rnd(minimum(locals[2]),rounding)*", "*rnd(maximum(locals[2]),rounding)*")")
        write(f," & ")
        write(f,"("*rnd(minimum(perturbations[2]),rounding)*", "*rnd(maximum(perturbations[2]),rounding)*")")
        write(f," & ")
        if i<=length(dwell_times)
            write(f,string(real_round(maximum(dwell_times),rounding)))
        end
        write(f," & ")
        write(f,"\\\\\n")
    end
    
    write(f," \t\\hline")
    write(f,'\n')
    
end
function write_table_tail(f,mode,local_search_param)
    write(f,"\t\\multicolumn{8}{l}{\$^{\\mathrm{\\boldsymbol{a}}}\$: For initial solution, local search, and perturbation respectively.}\\\\\n")
    write(f,'\n')
    write(f,"\\caption{Results using 1 point move")
    if '2' in mode
        write(f," and 2 point move")
    end
    write(f,", selecting the top ")
    if local_search_param == 1
        write(f,"vehicle")
    else
        write(f,string(local_search_param)*" vehicles")
    end
    write(f," of the heuristic \\lq remove target from vehicle with largest tour cost\\rq }")
    write(f,'\n')
    write(f,"\\end{longtable}")
    write(f,'\n')
end