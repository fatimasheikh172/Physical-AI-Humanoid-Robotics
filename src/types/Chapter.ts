import { Exercise } from './Exercise';
import { Simulation } from './Simulation';

export interface Chapter {
  id: string;
  title: string;
  slug: string;
  content: string; // Markdown/MDX content
  order: number; // Sequential order in textbook
  estimatedTime: number; // Minutes to complete
  prerequisites: string[]; // Array of Chapter IDs
  difficulty: 'beginner' | 'intermediate' | 'advanced';
  language: string;
  exercises?: Exercise[];
  simulations?: Simulation[];
  createdAt: string; // ISO date string
  updatedAt: string; // ISO date string
}