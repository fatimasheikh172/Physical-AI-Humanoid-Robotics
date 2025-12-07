import React from 'react';
import clsx from 'clsx';

interface HeadingProps {
  children: React.ReactNode;
  level: 1 | 2 | 3 | 4 | 5 | 6;
  className?: string;
  id?: string;
}

const Heading: React.FC<HeadingProps> = ({ children, level, className = '', id }) => {
  const HeadingTag = `h${level}` as keyof JSX.IntrinsicElements;
  
  const headingClasses = clsx(
    'font-bold mb-4 mt-6 first:mt-0',
    {
      'text-4xl': level === 1,
      'text-3xl': level === 2,
      'text-2xl': level === 3,
      'text-xl': level === 4,
      'text-lg': level === 5,
      'text-base': level === 6,
    },
    className
  );

  return (
    <HeadingTag id={id} className={headingClasses}>
      {children}
    </HeadingTag>
  );
};

interface ParagraphProps {
  children: React.ReactNode;
  className?: string;
  variant?: 'default' | 'lead' | 'small' | 'muted';
}

const Paragraph: React.FC<ParagraphProps> = ({ children, className = '', variant = 'default' }) => {
  const paragraphClasses = clsx(
    'mb-4',
    {
      'text-lg': variant === 'lead',
      'text-sm': variant === 'small',
      'text-gray-600 dark:text-gray-400': variant === 'muted',
    },
    className
  );

  return <p className={paragraphClasses}>{children}</p>;
};

interface CodeBlockProps {
  children: React.ReactNode;
  language?: string;
  className?: string;
  showLineNumbers?: boolean;
}

const CodeBlock: React.FC<CodeBlockProps> = ({ 
  children, 
  language = '', 
  className = '', 
  showLineNumbers = false 
}) => {
  const codeClasses = clsx(
    'block p-4 rounded-lg bg-gray-800 text-gray-100 font-mono text-sm overflow-x-auto',
    className
  );
  
  return (
    <pre className={codeClasses}>
      <code className="whitespace-pre-wrap break-words">{children}</code>
    </pre>
  );
};

interface InlineCodeProps {
  children: React.ReactNode;
  className?: string;
}

const InlineCode: React.FC<InlineCodeProps> = ({ children, className = '' }) => {
  return (
    <code className={clsx(
      'px-2 py-1 rounded bg-gray-100 dark:bg-gray-800 text-blue-600 dark:text-blue-400 font-mono text-sm',
      className
    )}>
      {children}
    </code>
  );
};

interface BlockquoteProps {
  children: React.ReactNode;
  className?: string;
}

const Blockquote: React.FC<BlockquoteProps> = ({ children, className = '' }) => {
  return (
    <blockquote className={clsx(
      'border-l-4 border-blue-500 pl-4 py-2 my-4 italic text-gray-700 dark:text-gray-300',
      className
    )}>
      {children}
    </blockquote>
  );
};

export { Heading, Paragraph, CodeBlock, InlineCode, Blockquote };