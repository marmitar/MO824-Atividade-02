% aga-homework.cls
%
% A LaTeX2e document class for preparing homework solutions.

% --- METADATA -----------------------------------------------------------------
\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{homework}

% --- OPTION PROCESSSING -------------------------------------------------------
\RequirePackage{kvoptions}
\SetupKeyvalOptions{
    family = homework,
    prefix = homework@
}

\DeclareStringOption[1]%
    {assignment}

% pass unrecognized options to `article` class
\DeclareDefaultOption{%
    \PassOptionsToClass{\CurrentOption}{article}%
}

% process all options
\ProcessKeyvalOptions{homework}

% load parent class with some default options
\LoadClass[a4paper, 11pt]{article}

\RequirePackage[brazilian]{babel}
\RequirePackage[T1]{fontenc}
\RequirePackage[utf8]{inputenc}
% --- PACKAGES AND SETUP -------------------------------------------------------
\RequirePackage[
    a4paper,
    headheight = 20pt,
    margin = 1in,
    top = \dimexpr 1in - 10pt \relax,
    bottom = \dimexpr 1in - 20pt \relax,
    footskip = 0pt,
]{geometry} % set up nicer margins
\addtolength{\footskip}{20pt}
\RequirePackage{fancyhdr} % for headers and footers
\RequirePackage{graphicx} % for including figures
\RequirePackage{mathpazo} % use Palation font
\RequirePackage{amsmath} % use AMS math package
\RequirePackage{amssymb} % use AMS symbols
\RequirePackage{amsthm} % for writing proofs
\RequirePackage{array} % for setting up arguments to columns
\RequirePackage{booktabs} % for professional tables
\RequirePackage{xcolor} % for colored text (comments in algorithms)
\RequirePackage{trimspaces, xstring} % for multiple author parsing
\RequirePackage[bottom]{footmisc}
\RequirePackage[hidelinks]{hyperref}

% look for figures in the figures directory
% \graphicspath{{figures/}}

% set up headers and footers; same for all pages
\setlength{\headheight}{14pt}
\fancyhf{}
\fancyhead[L]{\sffamily Universidade Estadual de Campinas}
\fancyhead[R]{\sffamily MC859 Projeto em Teoria da Computação, 2022 1S}
\fancyfoot[L]{\sffamily IC Instituto de Computação}
\fancyfoot[R]{\sffamily\bfseries\thepage}
\renewcommand{\headrulewidth}{0.5pt}
\renewcommand{\footrulewidth}{0.5pt}
\pagestyle{fancy}

\fancypagestyle{footnotes}{%
    \setlength{\footskip}{20pt}
}
\setlength{\skip\footins}{24pt plus 3pt minus 3pt}
\fancypagestyle{main}{%
    \setlength{\footskip}{44pt plus 3pt minus 3pt}
}
\renewcommand{\footnotelayout}{%
    \thispagestyle{footnotes}
}

\newcommand{\homework@student}[4][0.3\textwidth]{%
    \begin{minipage}{#1}
        \normalsize\centering
        % #2 ~(#3) \\
        #2 \\
        {\small RA~ #3} \\
        \href{mailto:#4}{\texttt{#4}}
    \end{minipage}%
}

% ensure that the title is inserted in the document
\title{\vspace{-2\baselineskip}Atividade \homework@assignment}
% \author{\aga@authorsProcessed}
\author{
    \hfill%
    \homework@student
        {Lucas Pedroso Cantanhede}{202058}
        {lpedrosoc@gmail.com}%
    \homework@student
        {Tiago de Paula Alves}{187679}
        {tiagodepalves@gmail.com}%
    \homework@student
        {Rafael Marques Miorim}{157065}
        {rmmiorim@hotmail.com}%
    \hfill
}
\AtBeginDocument{\maketitle\thispagestyle{fancy}}
