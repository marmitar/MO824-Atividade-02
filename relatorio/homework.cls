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

% ensure that the title is inserted in the document
\title{\vspace{-2\baselineskip}Atividade \homework@assignment}
% \author{\aga@authorsProcessed}
\author{
    \hfill%
    \begin{minipage}{0.4\textwidth}
        \centering
        Tiago de Paula Alves ~(187679) \\
        \href{mailto:tiagodepalves@gmail.com}{\texttt{tiagodepalves@gmail.com}}
    \end{minipage}%
    \begin{minipage}{0.4\textwidth}
        \centering
        Vinicius da Silva ~(206734) \\
        \href{mailto:v206734@dac.unicamp.br}{\texttt{v206734@dac.unicamp.br}}
    \end{minipage}%
    \hfill
}
\AtBeginDocument{\maketitle\thispagestyle{fancy}}
