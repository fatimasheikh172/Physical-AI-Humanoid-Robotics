import React from 'react';
import { Chapter } from '../../types/Chapter';
import { Progress } from '../../types/Progress';
import Link from '@docusaurus/Link';
import clsx from 'clsx';

interface ChapterNavigationSidebarProps {
  chapters: Chapter[];
  progress?: Progress[];
  currentChapterId?: string;
  className?: string;
}

const ChapterNavigationSidebar: React.FC<ChapterNavigationSidebarProps> = ({
  chapters = [],
  progress = [],
  currentChapterId,
  className = ''
}) => {
  // Sort chapters by order
  const sortedChapters = [...chapters].sort((a, b) => a.order - b.order);

  // Function to get progress for a specific chapter
  const getChapterProgress = (chapterId: string) => {
    return progress.find(p => p.chapterId === chapterId);
  };

  return (
    <div className={clsx('bg-white dark:bg-gray-800 rounded-lg border border-gray-200 dark:border-gray-700 overflow-hidden', className)}>
      <div className="p-4 border-b border-gray-200 dark:border-gray-700">
        <h2 className="text-lg font-semibold text-gray-900 dark:text-white">Textbook Contents</h2>
      </div>
      
      <nav className="py-2">
        <ul className="space-y-1">
          {sortedChapters.map((chapter) => {
            const chapterProgress = getChapterProgress(chapter.id);
            const isCurrent = chapter.id === currentChapterId;
            
            // Determine progress indicator
            let progressIndicator = null;
            if (chapterProgress) {
              if (chapterProgress.status === 'completed') {
                progressIndicator = (
                  <span className="ml-2 text-green-500" title="Completed">
                    ✓
                  </span>
                );
              } else if (chapterProgress.status === 'in-progress') {
                progressIndicator = (
                  <span className="ml-2 text-blue-500" title="In Progress">
                    ●
                  </span>
                );
              }
            }
            
            return (
              <li key={chapter.id}>
                <Link
                  to={`/docs/${chapter.slug}`}
                  className={clsx(
                    'flex items-center justify-between px-4 py-2 text-sm transition-colors',
                    isCurrent
                      ? 'bg-blue-50 dark:bg-blue-900/30 text-blue-700 dark:text-blue-300 font-medium'
                      : 'text-gray-700 hover:bg-gray-50 dark:text-gray-300 dark:hover:bg-gray-700/50'
                  )}
                >
                  <div className="flex items-center">
                    <span className={clsx(
                      'mr-2',
                      isCurrent ? 'text-blue-500' : 'text-gray-400'
                    )}>
                      {chapter.order}.
                    </span>
                    <span>{chapter.title}</span>
                  </div>
                  {progressIndicator}
                </Link>
              </li>
            );
          })}
        </ul>
      </nav>
      
      <div className="p-4 border-t border-gray-200 dark:border-gray-700">
        <div className="text-xs text-gray-500 dark:text-gray-400 mb-2">Progress Summary</div>
        <div className="w-full bg-gray-200 dark:bg-gray-700 rounded-full h-2">
          <div 
            className="bg-blue-600 h-2 rounded-full" 
            style={{ 
              width: `${(progress.filter(p => p.status === 'completed').length / chapters.length) * 100 || 0}%` 
            }}
          ></div>
        </div>
        <div className="text-xs text-gray-500 dark:text-gray-400 mt-1">
          {progress.filter(p => p.status === 'completed').length} of {chapters.length} chapters completed
        </div>
      </div>
    </div>
  );
};

export default ChapterNavigationSidebar;