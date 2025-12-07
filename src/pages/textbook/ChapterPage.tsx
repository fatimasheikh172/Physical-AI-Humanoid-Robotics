import React, { JSX, useEffect, useState } from 'react';
import Layout from '@site/src/components/common/Layout';
import ChapterNavigationSidebar from '@site/src/components/common/ChapterNavigationSidebar';
import { Chapter } from '@site/src/types/Chapter';
import { Progress } from '@site/src/types/Progress';
import { Heading, Paragraph, CodeBlock, InlineCode, Blockquote } from '@site/src/components/common/Typography';
// Ensure the correct path to AITutor component
import AITutor from '@site/frontend/src/components/ai-tutor/AITutor'; // Verify this path or create the file if missing
import { } from 'react-router-dom';
import LoadingError from '@site/src/components/common/LoadingError';

const ChapterPage: React.FC = () => {
  const { slug } = useParams<{ slug: string }>();
  const [chapter, setChapter] = useState<Chapter | null>(null);
  const [progress, setProgress] = useState<Progress | null>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [allChapters, setAllChapters] = useState<Chapter[]>([]);

  // Fetch all chapters (in a real app, this would come from your docs or API)
  useEffect(() => {
    const fetchChapters = async () => {
      try {
        // In a real implementation, we would fetch from the API
        // For now, we'll use mock data
        const mockChapters: Chapter[] = [
          {
            id: '1',
            title: 'Physical AI Foundation',
            slug: 'physical-ai-foundation',
            content: '# Physical AI Foundation\n\nPhysical AI is an interdisciplinary field that combines robotics, machine learning, and control theory to create machines capable of intelligent physical interaction with the world.\n\n## Key Concepts\n\n- Embodied Intelligence\n- Sensorimotor Learning\n- World Models\n\nThis is a foundational chapter that introduces the core concepts of Physical AI.',
            order: 1,
            estimatedTime: 30,
            prerequisites: [],
            difficulty: 'beginner',
            language: 'en',
            createdAt: new Date().toISOString(),
            updatedAt: new Date().toISOString()
          },
          {
            id: '2',
            title: 'ROS 2',
            slug: 'ros2',
            content: '# ROS 2\n\nRobot Operating System 2 (ROS 2) is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.\n\n## Key Features\n\n- Improved security\n- Better real-time support\n- Cross-platform compatibility\n\nROS 2 is essential for robotics development.',
            order: 2,
            estimatedTime: 45,
            prerequisites: ['1'],
            difficulty: 'intermediate',
            language: 'en',
            createdAt: new Date().toISOString(),
            updatedAt: new Date().toISOString()
          },
          {
            id: '3',
            title: 'Gazebo & Utility',
            slug: 'gazebo',
            content: '# Gazebo & Utility\n\nGazebo is a robotics simulator that provides realistic rendering, physics simulation, and sensor simulation. It is widely used in robotics research and development.\n\n## Key Components\n\n- Physics engine\n- Sensor simulation\n- Rendering engine\n\nSimulators like Gazebo are crucial for testing robotics algorithms before deployment.',
            order: 3,
            estimatedTime: 50,
            prerequisites: ['2'],
            difficulty: 'intermediate',
            language: 'en',
            createdAt: new Date().toISOString(),
            updatedAt: new Date().toISOString()
          }
        ];
        
        setAllChapters(mockChapters);
        
        // Find the specific chapter based on slug
        const foundChapter = mockChapters.find(ch => ch.slug === slug);
        if (foundChapter) {
          setChapter(foundChapter);
        } else {
          setError(`Chapter with slug "${slug}" not found`);
        }
      } catch (err) {
        setError('Failed to load chapters');
        console.error('Error fetching chapters:', err);
      } finally {
        setLoading(false);
      }
    };

    fetchChapters();
  }, [slug]);

  // Update progress when chapter changes
  useEffect(() => {
    if (chapter) {
      // In a real app, we would fetch user progress from the API
      // For now, we'll create mock progress
      const mockProgress: Progress = {
        id: `progress-${chapter.id}`,
        userId: 'mock-user-id',
        chapterId: chapter.id,
        status: 'in-progress',
        completionPercentage: 0,
        timeSpent: 0,
        lastAccessed: new Date().toISOString(),
        exercisesCompleted: 0,
        exercisesTotal: chapter.exercises ? chapter.exercises.length : 0,
        simulationsInteracted: 0,
        createdAt: new Date().toISOString(),
        updatedAt: new Date().toISOString()
      };
      
      setProgress(mockProgress);
    }
  }, [chapter]);

  if (loading) {
    return <LoadingError loading={true} loadingText="Loading chapter content..." />;
  }

  if (error) {
    return <LoadingError error={error} onRetry={() => window.location.reload()} />;
  }

  if (!chapter) {
    return <LoadingError error={`Chapter not found: ${slug}`} />;
  }

  // Parse chapter content and render it
  const renderContent = () => {
    // This is a simple implementation - in a real app, you'd want to use a proper Markdown parser
    // For now, we'll implement basic parsing of headings, paragraphs, and code blocks
    
    const content = chapter.content;
    const lines = content.split('\n');
    const elements: JSX.Element[] = [];
    let currentParagraph = '';
    let inCodeBlock = false;
    let codeBlockContent = '';
    let codeBlockLanguage = '';

    lines.forEach((line, index) => {
      if (line.startsWith('## ')) {
        // Add any pending paragraph before the heading
        if (currentParagraph) {
          elements.push(<Paragraph key={`p-${index}`}>{currentParagraph}</Paragraph>);
          currentParagraph = '';
        }
        const headingText = line.substring(3);
        elements.push(<Heading key={`h2-${index}`} level={2}>{headingText}</Heading>);
      } else if (line.startsWith('# ')) {
        // Add any pending paragraph before the heading
        if (currentParagraph) {
          elements.push(<Paragraph key={`p-${index}`}>{currentParagraph}</Paragraph>);
          currentParagraph = '';
        }
        const headingText = line.substring(2);
        elements.push(<Heading key={`h1-${index}`} level={1}>{headingText}</Heading>);
      } else if (line.startsWith('```')) {
        if (!inCodeBlock) {
          // Starting a code block
          inCodeBlock = true;
          codeBlockLanguage = line.substring(3);
          codeBlockContent = '';
        } else {
          // Ending a code block
          inCodeBlock = false;
          elements.push(
            <CodeBlock 
              key={`code-${index}`} 
              language={codeBlockLanguage}
            >
              {codeBlockContent}
            </CodeBlock>
          );
          codeBlockContent = '';
        }
      } else if (inCodeBlock) {
        // Inside a code block
        codeBlockContent += line + '\n';
      } else if (line.startsWith('> ')) {
        // Blockquote
        if (currentParagraph) {
          elements.push(<Paragraph key={`p-${index}`}>{currentParagraph}</Paragraph>);
          currentParagraph = '';
        }
        const quoteText = line.substring(2);
        elements.push(<Blockquote key={`quote-${index}`}>{quoteText}</Blockquote>);
      } else if (line.trim() === '') {
        // Empty line - finalize paragraph
        if (currentParagraph) {
          elements.push(<Paragraph key={`p-${index}`}>{currentParagraph}</Paragraph>);
          currentParagraph = '';
        }
      } else {
        // Regular paragraph content
        if (currentParagraph) {
          currentParagraph += ' ' + line;
        } else {
          currentParagraph = line;
        }
      }
    });

    // Add the final paragraph if there is one
    if (currentParagraph) {
      elements.push(<Paragraph key="final-p">{currentParagraph}</Paragraph>);
    }

    return elements;
  };

  return (
    <Layout 
      variant="sidebar"
      sidebarContent={
        <ChapterNavigationSidebar 
          chapters={allChapters} 
          currentChapterId={chapter.id}
        />
      }
      className="bg-gray-50 dark:bg-gray-900"
    >
      <div className="max-w-4xl mx-auto bg-white dark:bg-gray-800 rounded-lg shadow p-6">
        <header className="mb-6 border-b border-gray-200 dark:border-gray-700 pb-4">
          <h1 className="text-3xl font-bold text-gray-900 dark:text-white">{chapter.title}</h1>
          <div className="flex items-center mt-2 text-sm text-gray-600 dark:text-gray-400">
            <span>Difficulty: <span className="capitalize">{chapter.difficulty}</span></span>
            <span className="mx-2">•</span>
            <span>Estimated time: {chapter.estimatedTime} min</span>
          </div>
        </header>
        
        <main className="prose prose-gray dark:prose-invert max-w-none">
          {renderContent()}
        </main>
        
        <footer className="mt-8 border-t border-gray-200 dark:border-gray-700 pt-4 flex justify-between items-center">
          <div className="text-sm text-gray-600 dark:text-gray-400">
            Chapter {chapter.order} of {allChapters.length}
          </div>
          <button className="px-4 py-2 bg-blue-600 text-white rounded hover:bg-blue-700 transition-colors">
            Mark as Complete
          </button>
        </footer>
      </div>
      
      <div className="fixed top-1/2 right-4 transform -translate-y-1/2 z-10">
        <AITutor context={JSON.stringify({ chapterId: chapter.id, chapterTitle: chapter.title })} />
      </div>
    </Layout>
  );
};

export default ChapterPage;


function useParams<T>(): { slug: any; } {
  throw new Error('Function not implemented.');
}
// Removed the conflicting local useParams function
