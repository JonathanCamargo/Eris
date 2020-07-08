function tabledata=ReadBin(filename,varargin)
% Reads a binary file containing float data and returns a table.
% File must contain an csv ascii header.
% 
% 
% Parameters:
% 'TermChar': character for termination of data
% 'Format' :  type of data contained in the file (e.g. {'float','uint8_t'})

p=inputParser();
p.addParameter('TermChar',{13,newline});
p.addParameter('Format','float');

p.parse(varargin{:});

termChars=p.Results.TermChar;
format=p.Results.Format;

if iscell(termChars)
    for i=1:numel(termChars)
        if ischar(termChars{i})
            termChars{i}=sprintf(termChars{i});        
        end
    end
elseif ~iscell(termChars)
    if ischar(termChars)
            termChars={sprintf(termChars)};    
    else
            termChars={termChars};
    end
end


fid=fopen(filename,'r');
A=fread(fid);
fclose(fid);
cond=false(size(A));
for i=1:numel(termChars)
    cond=(cond | A==termChars{i});
end

endOfHeaderIdx=(find(cond,1));
header=char(A(1:endOfHeaderIdx-1)');
header=regexprep(header,'(\(\w*\))','');
header=strtrim(header);
data=A(endOfHeaderIdx+numel(termChars):end);
columns_str=strsplit(header,',');
columns=numel(columns_str);

%% Make the format according to the label in format
if ~iscell(format)
    format=repmat({format},1,columns);
end
%Column numbers according to format and split to cell array
sizes=zeros(1,numel(format));
for i=1:numel(format)
   switch format{i}
       case 'float'
           sizes(i)=4;           
       case 'uint8_t'
           sizes(i)=1;          
       otherwise
           error('Format (%s) not supported',format{i});   
   end
end
%%
%Reshape the data according to the number of termination characters and size
bindata=reshape(data,sum(sizes)+numel(termChars),[])';
%Reshape the data into cells
celldata=mat2cell(bindata,ones(size(bindata,1),1),[sizes ones(1,numel(termChars))]);
celldata=celldata(:,1:columns);

X=zeros(size(celldata));
for i=1:size(celldata,2)
    switch format{i}
        case 'float'
            fun=@(x)(double(typecast(uint8(x),'single')));            
        case 'uint8_t'
            fun=@(x)(typecast(uint8(x),'uint8'));            
    end
    value=cellfun(fun,celldata(:,i));
    X(:,i)=value;
end

%% Save as table with header
tabledata=array2table(X,'VariableNames',columns_str);
end