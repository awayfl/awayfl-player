import { b2Contact } from '../Contacts';

/**
* @private
*/
export class b2ContactRegister {
	public createFcn: Function; // fcn pointer
	public destroyFcn: Function;// fcn pointer
	public primary: boolean;
	public pool: b2Contact;
	public poolCount: number /** int */;
}