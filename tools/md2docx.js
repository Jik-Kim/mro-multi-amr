const SC='/tmp/claude-1000/-home-byeongjik-Kn1-proj/b6d0f225-650b-4bbe-9ae9-e73c49d662df/scratchpad/node_modules/docx';
const D=require(SC);
const {Document,Packer,Paragraph,TextRun,Table,TableRow,TableCell,WidthType,ShadingType,
       HeadingLevel,AlignmentType,BorderStyle,PageOrientation,LevelFormat,PageNumber,
       Header,Footer,convertMillimetersToTwip}=D;
const fs=require('fs');

const SRC=process.argv[2], OUT=process.argv[3];
const FONT='Noto Sans CJK KR';
const MONO='NanumGothicCoding';        // 인라인 `코드`
const MONOBLK='DejaVu Sans Mono';      // 도식 블록 — 박스문자 폭이 한글의 정확히 절반이라 정렬이 맞는다
const PAGE_W=11906, MARGIN=992;
const USABLE=PAGE_W-MARGIN*2;              // 9922 DXA
const BODY=20;                             // 10pt (half-points)
const TBL=17;                              // 8.5pt

// ── 인라인 파서 (재귀) — **굵게** `코드` ~~취소선~~ *기울임* 중첩 처리
function runs(text, st={}) {
  const out=[];
  const re=/(\*\*[\s\S]+?\*\*|`[^`]+`|~~[\s\S]+?~~|\*[^*\n]+?\*)/g;
  const size=st.size||BODY;
  const plain=t=>{ if(t) out.push(new TextRun({text:t,font:st.mono?MONO:FONT,size,
      bold:!!st.bold, italics:!!st.italics, strike:!!st.strike,
      color:st.mono?'8A4B08':(st.color||undefined)})); };
  let last=0,m;
  while((m=re.exec(text))!==null){
    plain(text.slice(last,m.index));
    const tk=m[0];
    if(tk.startsWith('**'))      out.push(...runs(tk.slice(2,-2), {...st,bold:true}));
    else if(tk.startsWith('~~')) out.push(...runs(tk.slice(2,-2), {...st,strike:true}));
    else if(tk.startsWith('`'))  out.push(new TextRun({text:tk.slice(1,-1),font:MONO,size,
                                    bold:!!st.bold,strike:!!st.strike,color:'8A4B08'}));
    else                          out.push(...runs(tk.slice(1,-1), {...st,italics:true}));
    last=m.index+tk.length;
  }
  plain(text.slice(last));
  return out.length?out:[new TextRun({text:'',font:FONT,size})];
}

const md=fs.readFileSync(SRC,'utf8').split('\n');
const children=[];
let i=0;

const splitRow=l=>l.trim().replace(/^\|/,'').replace(/\|$/,'').split('|').map(c=>c.trim());
const isSep=l=>/^\|[\s:|-]+\|?$/.test(l.trim());

function widths(rows,cols){
  const len=new Array(cols).fill(1), word=new Array(cols).fill(1);
  rows.forEach(r=>r.forEach((c,k)=>{
    if(k>=cols) return;
    len[k]=Math.max(len[k], Math.min(c.length,150));
    // 공백 없는 최장 토큰 — 이보다 열이 좁으면 낱말이 강제로 쪼개진다
    c.replace(/[*`~]/g,'').split(/\s+/).forEach(t=>{ if(t.length>word[k]) word[k]=t.length; });
  }));
  // 토큰이 들어갈 최소 폭 (8.5pt 기준 글자당 ≈95 DXA + 여백), 열당 상한을 둬 한 열이 표를 먹지 않게
  const capPer=Math.floor(USABLE/cols*1.9);
  const minw=word.map(v=>Math.min(capPer, Math.max(Math.floor(USABLE/cols*0.45), v*95+200)));
  const tot=len.reduce((a,b)=>a+b,0);
  let w=len.map(v=>Math.round(USABLE*v/tot));
  for(let pass=0; pass<12; pass++){
    if(w.every((v,k)=>v>=minw[k])) break;
    w=w.map((v,k)=>v<minw[k]?minw[k]:v);
    const over=w.reduce((a,b)=>a+b,0)-USABLE;
    if(over<=0) break;
    const slack=w.map((v,k)=>Math.max(0,v-minw[k]));
    const st=slack.reduce((a,b)=>a+b,0);
    if(st<=0) break;
    w=w.map((v,k)=>v-Math.round(over*slack[k]/st));
  }
  const d=USABLE-w.reduce((a,b)=>a+b,0);
  w[w.indexOf(Math.max(...w))]+=d;
  return w.map(v=>Math.max(300,v));
}

while(i<md.length){
  const line=md[i];

  if(/^\|/.test(line) && i+1<md.length && isSep(md[i+1])){
    const head=splitRow(line); const cols=head.length;
    const body=[]; i+=2;
    while(i<md.length && /^\|/.test(md[i])){ body.push(splitRow(md[i])); i++; }
    const cw=widths([head,...body],cols);
    const mkCell=(txt,isHead,k)=>new TableCell({
      width:{size:cw[k],type:WidthType.DXA},
      shading:isHead?{type:ShadingType.CLEAR,fill:'E8EDF3'}:undefined,
      margins:{top:40,bottom:40,left:80,right:80},
      children:[new Paragraph({spacing:{before:0,after:0,line:250},
        children:runs(txt,{size:TBL,bold:isHead})})]});
    const row=(cells,isHead)=>new TableRow({tableHeader:isHead,cantSplit:true,
      children:Array.from({length:cols},(_,k)=>mkCell(cells[k]||'',isHead,k))});
    children.push(new Table({
      columnWidths:cw, width:{size:USABLE,type:WidthType.DXA},
      borders:{top:{style:BorderStyle.SINGLE,size:4,color:'AAB4C0'},
               bottom:{style:BorderStyle.SINGLE,size:4,color:'AAB4C0'},
               left:{style:BorderStyle.SINGLE,size:4,color:'AAB4C0'},
               right:{style:BorderStyle.SINGLE,size:4,color:'AAB4C0'},
               insideHorizontal:{style:BorderStyle.SINGLE,size:2,color:'C8D0DA'},
               insideVertical:{style:BorderStyle.SINGLE,size:2,color:'C8D0DA'}},
      rows:[row(head,true),...body.map(r=>row(r,false))]}));
    children.push(new Paragraph({spacing:{after:160},children:[]}));
    continue;
  }

  if(/^```/.test(line)){
    i++; const buf=[];
    while(i<md.length && !/^```/.test(md[i])){ buf.push(md[i]); i++; }
    i++;
    buf.forEach(t=>children.push(new Paragraph({
      spacing:{before:0,after:0,line:230},
      shading:{type:ShadingType.CLEAR,fill:'F4F6F8'},
      children:[new TextRun({text:t||' ',font:MONOBLK,size:15})]})));
    children.push(new Paragraph({spacing:{after:140},children:[]}));
    continue;
  }

  if(/^#{1,4} /.test(line)){
    const lv=line.match(/^#+/)[0].length;
    const txt=line.replace(/^#+\s*/,'');
    const H=[HeadingLevel.TITLE,HeadingLevel.HEADING_1,HeadingLevel.HEADING_2,HeadingLevel.HEADING_3][lv-1];
    children.push(new Paragraph({heading:H,
      spacing:{before:lv===1?0:(lv===2?320:240),after:lv===1?80:140},
      children:runs(txt,{size:lv===1?36:(lv===2?26:22),bold:true,color:'12294D'})}));
    i++; continue;
  }

  if(/^>\s?/.test(line)){
    const buf=[];
    while(i<md.length && /^>\s?/.test(md[i])){ buf.push(md[i].replace(/^>\s?/,'')); i++; }
    const paras=[]; let cur='';
    buf.forEach(t=>{ if(t.trim()===''){ if(cur){paras.push(cur);cur='';} } else cur = cur? cur+' '+t.trim() : t.trim(); });
    if(cur) paras.push(cur);
    paras.forEach(t=>children.push(new Paragraph({
      spacing:{before:0,after:60,line:270},
      indent:{left:280},
      border:{left:{style:BorderStyle.SINGLE,size:12,color:'8FA6C4',space:10}},
      children:runs(t,{})})));
    children.push(new Paragraph({spacing:{after:120},children:[]}));
    continue;
  }

  if(/^---+\s*$/.test(line)){
    children.push(new Paragraph({spacing:{before:60,after:160},
      border:{bottom:{style:BorderStyle.SINGLE,size:6,color:'C8D0DA',space:6}},children:[]}));
    i++; continue;
  }

  // 이어지는 줄인가 — 표·제목·인용·구분선·목록·코드블록의 시작이 아니면 앞 문단에 붙는다
  const cont=t=>t!==undefined && t.trim()!=='' && !/^\|/.test(t) && !/^#{1,4} /.test(t)
      && !/^>\s?/.test(t) && !/^---+\s*$/.test(t) && !/^\s*[-*] /.test(t) && !/^```/.test(t);

  if(/^\s*[-*] /.test(line)){
    const ind=line.match(/^\s*/)[0].length;
    let txt=line.replace(/^\s*[-*] /,''); i++;
    while(cont(md[i])){ txt+=' '+md[i].trim(); i++; }
    children.push(new Paragraph({numbering:{reference:'bl',level:ind>=2?1:0},
      spacing:{before:0,after:60,line:270},children:runs(txt,{})}));
    continue;
  }

  if(line.trim()===''){ i++; continue; }

  let para=line.trim(); i++;
  while(cont(md[i])){ para+=' '+md[i].trim(); i++; }
  children.push(new Paragraph({spacing:{before:0,after:120,line:280},children:runs(para,{})}));
}

const doc=new Document({
  styles:{default:{document:{run:{font:FONT,size:BODY,color:'1A1A1A'}}}},
  numbering:{config:[{reference:'bl',levels:[
    {level:0,format:LevelFormat.BULLET,text:'•',alignment:AlignmentType.LEFT,
     style:{paragraph:{indent:{left:340,hanging:200}}}},
    {level:1,format:LevelFormat.BULLET,text:'–',alignment:AlignmentType.LEFT,
     style:{paragraph:{indent:{left:640,hanging:200}}}}]}]},
  sections:[{
    properties:{page:{size:{width:PAGE_W,height:16838,orientation:PageOrientation.PORTRAIT},
      margin:{top:935,bottom:907,left:MARGIN,right:MARGIN,header:397,footer:397}}},
    footers:{default:new Footer({children:[new Paragraph({alignment:AlignmentType.CENTER,
      children:[new TextRun({children:[PageNumber.CURRENT],font:FONT,size:16,color:'6B7280'})]})]})},
    children}]});

Packer.toBuffer(doc).then(b=>{fs.writeFileSync(OUT,b);console.log('wrote',OUT,b.length,'bytes','blocks:',children.length);});
