%%
%% jctvcdoc: Copyright (c) 2011 BBC Research & Development
%% All rights reserved.
%%
%% Permission to use, copy, modify, and/or distribute this software for any
%% purpose with or without fee is hereby granted, provided that the above
%% copyright notice and this permission notice appear in all copies.
%%
%% Neither the names of the BBC, ITU, ISO, IEC nor the names of its
%% contributors may be used to endorse or promote products derived from
%% this software without specific prior written permission.
%%
%% THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
%% WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
%% MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
%% ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
%% WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
%% ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
%% OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
%%
\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{jctvcdoc}[2011/06/01 JCT-VC document class]

\LoadClassWithOptions{article}

% typeset using Times New Roman
\RequirePackage{ifxetex}
\ifxetex
  % use the correct fonts when using XeTeX
  \RequirePackage{fontspec}
  \defaultfontfeatures{Mapping=tex-text}
  \setmainfont{Times New Roman}
\else
  \RequirePackage{times}
\fi

% expand the margins
\RequirePackage{geometry}
\geometry{tmargin=1.6cm,lmargin=1in,rmargin=2.5cm,bmargin=1in,nohead}

% require graphicx for loading the logos in the header
\RequirePackage{graphicx}

% don't number the abstract
\renewenvironment{abstract}{\section*{Abstract}}{}

% format the date in iso style
\RequirePackage{datetime}
\renewcommand{\dateseparator}{-}
\newdateformat{JCTVC@yyyymmdddate}{%
	\THEYEAR\dateseparator\twodigit{\THEMONTH}\dateseparator\twodigit{\THEDAY}}

% all pages only have a footer
\RequirePackage{fancyhdr}
\pagestyle{fancy}
\renewcommand{\headrulewidth}{0pt}
\fancyhead{}
\cfoot{\thepage}
\rfoot{Date saved: \JCTVC@yyyymmdddate\today}

% macros for document metadata
\RequirePackage[normalem]{ulem}
\def\@jctvcdocnum{\uline{xxx}}
\def\@jctvcdocstatus{\uline{TODO: Add \texttt{\textbackslash{}jctvcdocstatus}}}
\def\@jctvcdocpurpose{\uline{TODO: Add \texttt{\textbackslash{}jctvcdocpurpose}}}
\def\@jctvcdocsource{\uline{TODO: Add \texttt{\textbackslash{}jctvcdocsource}}}
\def\@jctvcmeeting{\uline{TODO: Add \texttt{\textbackslash{}jctvcmeeting}}}
\newcommand{\jctvcdocnum}[1]{\def\@jctvcdocnum{\mbox{#1}}}
\newcommand{\jctvcdocstatus}[1]{\def\@jctvcdocstatus{\mbox{#1}}}
\newcommand{\jctvcdocpurpose}[1]{\def\@jctvcdocpurpose{\mbox{#1}}}
\newcommand{\jctvcdocsource}[1]{\def\@jctvcdocsource{\mbox{#1}}}
\newcommand{\jctvcmeeting}[1]{\def\@jctvcmeeting{\mbox{#1}}}

% affiliation block in the author list
\newenvironment{affiliation}{%
	\begin{tabular}[t]{@{}>{\renewcommand{\\}{, }}l@{}}%
}{%
	\end{tabular}%
}

\RequirePackage{array}

\RequirePackage{ifthen}
\newcounter{jctvc@author@column}
\newcommand{\@jctvc@switch@author@column}{%
	\ifthenelse{\value{jctvc@author@column} = 1}{%
		% do nothing
	}{%
		\addtocounter{jctvc@author@column}{1}%
		&
	}%
}

\newcommand*{\email}[1]{\@jctvc@switch@author@column\textit{#1}}
\newcommand*{\tel}[1]{\@jctvc@switch@author@column\textrm{#1}}

% Make a title at the top of the current page.
\renewcommand\maketitle{%
	% none of the title is put in the header (it is too difficult to
	% make the page match the word style in this case)
	\vspace*{-1.7cm}
	\begin{raggedright}
	\rule{0pt}{0.74cm}% strut incase images aren't loaded
	\IfFileExists{logos/itu}{\includegraphics[height=0.74cm]{logos/itu}}{}
	\IfFileExists{logos/iso}{\includegraphics[height=0.74cm]{logos/iso}}{}
	\IfFileExists{logos/iec}{\includegraphics[height=0.74cm]{logos/iec}}{}
	\\
	\textbf{Joint Collaborative Team on Video Coding (JCT-VC)}\\[0ex]
	\textbf{of ITU-T SG16 WP3 and ISO/IEC JTC1/SC29/WG11}
		\hfill Document: JCTVC-\@jctvcdocnum\\[0ex]
	\@jctvcmeeting

	\vspace{0.5\baselineskip}

	\newcommand{\@strutb}{\rule{0pt}{2.5ex}}
	\begin{tabular}{lp{0.78\textwidth}}
	\@strutb \it Title: & \@title \\
	\@strutb \it Status: & \@jctvcdocstatus \\
	\@strutb \it Purpose: & \@jctvcdocpurpose \\
	\@strutb \it Author(s): & %
		\setcounter{jctvc@author@column}{0}
		\let\@and\\
		\renewcommand{\and}{\@and\setcounter{jctvc@author@column}{0}}
		\newcommand{\@NLtoPAR}{\renewcommand{\\}{\par}}
		\begin{tabular}[t]{@{}>{\@NLtoPAR}p{3in}|>{\@NLtoPAR}p{2in}@{}}%
			\@author
		\end{tabular}\\
	\@strutb \it Source: & \@jctvcdocsource \\
	\end{tabular}
	\end{raggedright}

	% draw a short horizontal line to delimit the title from body
	{\center\rule{0.35\textwidth}{1pt}\\}
}

%
\RequirePackage{parskip}
\RequirePackage[compact]{titlesec}
\titlespacing{\section}{0pt}{*0}{*0}
\titlespacing{\subsection}{0pt}{*0}{*0}
\titlespacing{\subsubsection}{0pt}{*0}{*0}
