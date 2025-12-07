import React from 'react';
import { Chapter, ChapterProgress } from '../../types/Chapter';
import { Link } from 'react-router-dom';

interface ChapterNavigationSidebarProps {
  chapters: Chapter[];
  progress: Record<string, ChapterProgress>;
  currentChapterId: string;
}

const ChapterNavigationSidebar: React.FC<ChapterNavigationSidebarProps> = ({ 
  chapters, 
  progress,
  currentChapterId
}) => {
  // Sort chapters by order
  const sortedChapters = [...chapters].sort((a, b) => a.order - b.order);

  return (
    <div className="bg-white shadow-md rounded-lg overflow-hidden">
      <div className="p-4 border-b">
        <h2 className="text-lg font-semibold text-gray-800">Textbook Contents</h2>
      </div>
      
      <nav className="py-2">
        <ul className="space-y-1 px-2">
          {sortedChapters.map((chapter) => {
            const chapterProgress = progress[chapter.id];
            const isCurrent = chapter.id === currentChapterId;
            
            return (
              <li key={chapter.id}>
                <Link
                  to={`/textbook/${chapter.slug}`}
                  className={`
                    flex items-center p-3 rounded-md text-sm transition-colors
                    ${isCurrent 
                      ? 'bg-blue-100 text-blue-800 font-medium' 
                      : 'text-gray-700 hover:bg-gray-100'}
                  `}
                >
                  <span className="mr-2 font-mono text-xs opacity-70">
                    {chapter.order}.
                  </span>
                  <span className="truncate">{chapter.title}</span>
                  
                  {chapterProgress && (
                    <div className="ml-auto flex items-center">
                      <div className="w-16 bg-gray-200 rounded-full h-1.5 ml-2">
                        <div 
                          className="bg-blue-600 h-1.5 rounded-full" 
                          style={{ width: `${chapterProgress.completionPercentage}%` }}
                        ></div>
                      </div>
                      <span className="text-xs text-gray-500 ml-2 w-8">
                        {chapterProgress.completionPercentage}%
                      </span>
                    </div>
                  )}
                </Link>
              </li>
            );
          })}
        </ul>
      </nav>
      
      <div className="p-4 border-t text-xs text-gray-500">
        {sortedChapters.length} chapters
      </div>
    </div>
  );
};

export default ChapterNavigationSidebar;