function verboseDisp(verbose,format,params)
%VERBOSEDISP Conditional printing. Useful for debug purposes.
%
%VERBOSEDISP(verbose,string) prints STRING iff verbose == true. Otherwise
%it does not print anything.
%
%VERBOSEDISP(...,params) prints FORMAT with the optional params, specified
%as a Nx1 vector iff verbose == true. Otherwise it does not print anything.

arguments
    verbose logical
    format string
    params = []
end

if verbose
    if nargin >= 3
        fprintf(format, params);
    else
        disp(format);
    end
end

end

