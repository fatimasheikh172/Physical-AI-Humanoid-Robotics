// Example MDX content for a chapter
// This would typically be in a .mdx file in the docs directory
// But we'll create a React component that renders MDX content

import React from 'react';
import { MDXProvider } from '@mdx-js/react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import { 
  Heading, 
  Paragraph, 
  CodeBlock, 
  InlineCode, 
  Blockquote 
} from '../common/Typography';
import { Card, CardContent } from '../common/Card';

// Custom MDX components mapping
const mdxComponents = {
  h1: (props: any) => <Heading level={1} {...props} />,
  h2: (props: any) => <Heading level={2} {...props} />,
  h3: (props: any) => <Heading level={3} {...props} />,
  h4: (props: any) => <Heading level={4} {...props} />,
  h5: (props: any) => <Heading level={5} {...props} />,
  h6: (props: any) => <Heading level={6} {...props} />,
  p: (props: any) => <Paragraph {...props} />,
  code: (props: any) => <InlineCode {...props} />,
  pre: (props: any) => {
    // Extract language from className if available
    const language = props.className?.match(/language-(\w+)/)?.[1] || 'text';
    return (
      <CodeBlock language={language}>
        {props.children}
      </CodeBlock>
    );
  },
  blockquote: (props: any) => <Blockquote {...props} />,
  strong: (props: any) => <strong className="font-bold" {...props} />,
  em: (props: any) => <em className="italic" {...props} />,
  ul: (props: any) => <ul className="list-disc list-inside mb-4" {...props} />,
  ol: (props: any) => <ol className="list-decimal list-inside mb-4" {...props} />,
  li: (props: any) => <li className="mb-1" {...props} />,
  a: (props: any) => (
    <a 
      {...props} 
      className="text-blue-600 hover:text-blue-800 dark:text-blue-400 dark:hover:text-blue-300 underline"
    />
  ),
  // Custom components for textbook-specific elements
  RobotArchitecture: ({ title, children }: { title: string; children: React.ReactNode }) => (
    <Card className="my-6 border-blue-500">
      <CardContent className="p-4">
        <h3 className="font-bold text-lg text-blue-700 dark:text-blue-300 mb-2">{title}</h3>
        <div>{children}</div>
      </CardContent>
    </Card>
  ),
  CodeExample: ({ 
    language = 'javascript', 
    children,
    caption
  }: { 
    language?: string; 
    children: React.ReactNode;
    caption?: string;
  }) => (
    <div className="my-6">
      {caption && <div className="text-sm text-gray-600 dark:text-gray-400 mb-1">{caption}</div>}
      <CodeBlock language={language}>
        {children}
      </CodeBlock>
    </div>
  ),
  ExerciseBox: ({ 
    type, 
    children 
  }: { 
    type: 'practice' | 'challenge' | 'quiz';
    children: React.ReactNode;
  }) => {
    const typeStyles = {
      practice: 'border-green-500 bg-green-50 dark:bg-green-900/20',
      challenge: 'border-yellow-500 bg-yellow-50 dark:bg-yellow-900/20',
      quiz: 'border-blue-500 bg-blue-50 dark:bg-blue-900/20'
    };

    const typeLabels = {
      practice: 'Practice Exercise',
      challenge: 'Challenge',
      quiz: 'Quiz Question'
    };

    return (
      <Card className={`my-6 border-2 ${typeStyles[type]}`}>
        <CardContent className="p-4">
          <h4 className="font-bold mb-2">{typeLabels[type]}</h4>
          <div>{children}</div>
        </CardContent>
      </Card>
    );
  },
};

interface MDXChapterRendererProps {
  children: React.ReactNode;
}

const MDXChapterRenderer: React.FC<MDXChapterRendererProps> = ({ children }) => {
  return (
    <MDXProvider components={mdxComponents}>
      <BrowserOnly>
        {() => children}
      </BrowserOnly>
    </MDXProvider>
  );
};

export default MDXChapterRenderer;