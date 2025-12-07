import apiClient from '../services/api';

interface TutorQuery {
  query: string;
  context?: {
    chapterId?: string;
    simulationId?: string;
    additionalContext?: string;
  };
}

interface TutorResponse {
  queryId: string;
  query: string;
  response: string;
  timestamp: string;
  context: any;
}

interface TutorRating {
  rating: number; // 1-5
}

class AITutorService {
  async sendQuery(query: TutorQuery): Promise<TutorResponse> {
    try {
      return await apiClient.post('/ai-tutor/query', query);
    } catch (error) {
      throw new Error(`AI Tutor query failed: ${(error as Error).message}`);
    }
  }

  async rateResponse(queryId: string, rating: number): Promise<void> {
    try {
      if (rating < 1 || rating > 5) {
        throw new Error('Rating must be between 1 and 5');
      }
      
      await apiClient.post(`/ai-tutor/${queryId}/rate`, { rating } as TutorRating);
    } catch (error) {
      throw new Error(`AI Tutor rating failed: ${(error as Error).message}`);
    }
  }
}

export default new AITutorService();