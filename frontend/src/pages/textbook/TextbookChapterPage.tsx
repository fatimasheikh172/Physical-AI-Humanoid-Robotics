import React, { useEffect, useState } from 'react';
import { useParams } from 'react-router-dom';
import { ChapterService, ProgressService } from '../../services/ApiClient';
import { Chapter, ChapterProgress } from '../../types/Chapter';
import ChapterCompletionTracker from '../textbook/ChapterCompletionTracker';
import ChapterNavigationSidebar from '../textbook/ChapterNavigationSidebar';
import AITutor from '../ai-tutor/AITutor';
import SimulationViewer from '../simulations/SimulationViewer';
import LoadingError from '../common/LoadingError';

const TextbookChapterPage: React.FC = () => {
  const { slug } = useParams<{ slug: string }>();
  const [chapter, setChapter] = useState<Chapter | null>(null);
  const [allChapters, setAllChapters] = useState<Chapter[]>([]);
  const [progress, setProgress] = useState<Record<string, ChapterProgress>>({});
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    const fetchData = async () => {
      try {
        setLoading(true);
        setError(null);

        // Fetch all chapters for navigation
        const chaptersResponse = await ChapterService.getAll();
        setAllChapters(chaptersResponse.data);

        // Find the specific chapter based on slug
        const targetChapter = chaptersResponse.data.find(c => c.slug === slug);
        
        if (!targetChapter) {
          throw new Error('Chapter not found');
        }

        setChapter(targetChapter);

        // Fetch progress for all chapters
        const progressResponse = await ProgressService.getAll();
        const progressMap: Record<string, ChapterProgress> = {};
        
        progressResponse.data.forEach(p => {
          progressMap[p.chapterId] = p;
        });
        
        setProgress(progressMap);
      } catch (err) {
        setError(err instanceof Error ? err.message : 'Failed to load chapter');
      } finally {
        setLoading(false);
      }
    };

    if (slug) {
      fetchData();
    }
  }, [slug]);

  const handleProgressUpdate = (updatedProgress: ChapterProgress) => {
    setProgress(prev => ({
      ...prev,
      [updatedProgress.chapterId]: updatedProgress
    }));
  };

  if (error) {
    return <LoadingError loading={false} error={error} onRetry={() => window.location.reload()} />;
  }

  if (loading || !chapter) {
    return <LoadingError loading={true} error={null} />;
  }

  return (
    <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8 py-8">
      <div className="flex flex-col lg:flex-row gap-8">
        {/* Sidebar with chapter navigation */}
        <div className="lg:w-1/4">
          <ChapterNavigationSidebar
            chapters={allChapters}
            progress={progress}
            currentChapterId={chapter.id}
          />
        </div>

        {/* Main content area */}
        <div className="lg:w-2/4">
          <article className="bg-white rounded-lg shadow p-6">
            <header className="mb-6">
              <h1 className="text-3xl font-bold text-gray-900 mb-2">{chapter.title}</h1>
              <div className="flex items-center text-sm text-gray-500">
                <span>Difficulty: {chapter.difficulty}</span>
                <span className="mx-2">•</span>
                <span>Estimated time: {chapter.estimatedTime} minutes</span>
              </div>
            </header>

            <div className="prose max-w-none">
              {/* In a real implementation, this would render MDX content */}
              <div dangerouslySetInnerHTML={{ __html: chapter.content.replace(/\n/g, '<br />') }} />
            </div>

            {/* Chapter completion tracker */}
            <div className="mt-8">
              <ChapterCompletionTracker
                chapterId={chapter.id}
                progress={progress[chapter.id] || null}
                onProgressUpdate={handleProgressUpdate}
              />
            </div>

            {/* Simulations for this chapter */}
            {chapter.simulations && chapter.simulations.length > 0 && (
              <div className="mt-8">
                <h2 className="text-xl font-semibold mb-4">Interactive Simulations</h2>
                <div className="space-y-6">
                  {chapter.simulations.map(simulation => (
                    <SimulationViewer key={simulation.id} simulation={simulation} />
                  ))}
                </div>
              </div>
            )}
          </article>
        </div>

        {/* AI Tutor sidebar */}
        <div className="lg:w-1/4">
          <AITutor context={chapter.title} chapter={chapter} />
        </div>
      </div>
    </div>
  );
};

export default TextbookChapterPage;