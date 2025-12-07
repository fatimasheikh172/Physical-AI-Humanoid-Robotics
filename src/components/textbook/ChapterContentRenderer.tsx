import React from 'react';
import { MDXProvider } from '@mdx-js/react';
import { 
  Heading, 
  Paragraph, 
  CodeBlock, 
  InlineCode, 
  Blockquote 
} from '../common/Typography';
import Button from '../common/Button';
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
  code: (props: any) => {
    // Check if this is a code block (with children containing newlines) or inline code
    if (props['data-parent-name'] === 'pre') {
      // This is a code block, already handled by pre element
      return <code {...props} />;
    }
    // This is inline code
    return <InlineCode {...props} />;
  },
  pre: (props: any) => {
    // Extract language from className if available
    const language = props.className?.match(/language-(\w+)/)?.[1] || '';
    return (
      <CodeBlock language={language} {...props}>
        {props.children.props.children}
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
  Button: (props: any) => <Button {...props} />,
  Card: (props: any) => <Card {...props} />,
  CardContent: (props: any) => <CardContent {...props} />,
};

interface ChapterContentRendererProps {
  content: string;
  components?: any;
}

const ChapterContentRenderer: React.FC<ChapterContentRendererProps> = ({ 
  content, 
  components = {} 
}) => {
  // In a Docusaurus environment, MDX content is typically processed at build time
  // This component would integrate with Docusaurus's MDX capabilities
  // For now, we'll use a simplified approach with the MDXProvider

  return (
    <MDXProvider components={{ ...mdxComponents, ...components }}>
      <div className="chapter-content">
        {content}
      </div>
    </MDXProvider>
  );
};

export default ChapterContentRenderer;

// Helper components for textbook-specific content

// For displaying robot architecture diagrams
export const RobotArchitecture = ({ title, children }: { title: string; children: React.ReactNode }) => (
  <Card className="my-6 border-blue-500">
    <CardContent className="p-4">
      <h3 className="font-bold text-lg text-blue-700 dark:text-blue-300 mb-2">{title}</h3>
      <div>{children}</div>
    </CardContent>
  </Card>
);

// For displaying code snippets with execution context
export const CodeExample = ({ 
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
);

// For displaying exercises within chapters
export const ExerciseBox = ({ 
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
};